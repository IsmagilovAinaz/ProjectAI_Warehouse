#include "WarehouseGameMode.h"
#include "SimulationDataCollector.h"
#include "Kismet/GameplayStatics.h"
#include "Blueprint/UserWidget.h"
#include "Engine/World.h"
#include "Misc/DateTime.h"
#include "Misc/Paths.h"
#include "HAL/PlatformFilemanager.h"
#include "ClickableRobotInterface.h"
#include "RobotStateVisualizer.h"
#include "PathFollowerComponent.h"
#include "Components/TextBlock.h"
#include "GameFramework/Character.h"                    
#include "GameFramework/CharacterMovementComponent.h"   
#include "SimulationHUDWidget.h"
#include "MAPFPlanner.h"


AWarehouseGameMode::AWarehouseGameMode()
{
    // Используем Spectator Pawn по умолчанию
    DefaultPawnClass = nullptr; // Будет использован SpectatorPawn из движка
}

void AWarehouseGameMode::BeginPlay()
{
    Super::BeginPlay();

    TArray<AActor*> FoundManagers;
    UGameplayStatics::GetAllActorsWithTag(GetWorld(), FName(TEXT("TaskManager")), FoundManagers);
    AActor* TaskManager = FoundManagers.Num() > 0 ? FoundManagers[0] : nullptr;


    APlayerController* PC = GetWorld()->GetFirstPlayerController();
    if (PC)
    {
        PC->bShowMouseCursor = true;
        PC->bEnableClickEvents = true;
        PC->bEnableMouseOverEvents = true;
    }

    // Задержка на 1 секунду перед привязкой
    FTimerHandle TimerHandle;
    GetWorldTimerManager().SetTimer(TimerHandle, this,
        &AWarehouseGameMode::BindRobotClicks, 1.0f, false);

    if (HUDWidgetClass)
    {
        USimulationHUDWidget* HUD = CreateWidget<USimulationHUDWidget>(GetWorld(), HUDWidgetClass);
        if (HUD)
        {
            HUD->TaskManager = TaskManager;  // Передать ссылку
            HUD->AddToViewport();
        }
    }

    UE_LOG(LogTemp, Log, TEXT("WarehouseGameMode: Ready. Click on robots."));
}

void AWarehouseGameMode::BindRobotClicks()
{
    TArray<AActor*> AllActors;
    UGameplayStatics::GetAllActorsOfClass(GetWorld(), AActor::StaticClass(), AllActors);

    int32 BoundCount = 0;
    for (AActor* Actor : AllActors)
    {
        // Проверяем, реализует ли актор наш интерфейс
        if (Actor->Implements<UClickableRobotInterface>())
        {
            Actor->OnClicked.AddDynamic(this, &AWarehouseGameMode::OnRobotClicked);
            BoundCount++;
        }
    }

    UE_LOG(LogTemp, Log, TEXT("WarehouseGameMode: Bound clicks to %d robots"), BoundCount);
}

void AWarehouseGameMode::OnRobotClicked(AActor* ClickedActor, FKey Button)
{
    if (ClickedActor && Button == EKeys::LeftMouseButton)
    {
        SelectRobot(ClickedActor);
    }
}

void AWarehouseGameMode::SelectRobot(AActor* Robot)
{
    if (!Robot || SelectedRobot == Robot) return;

    // 1. Снять предыдущее выделение
    DeselectRobot();

    // 2. Установить новое
    SelectedRobot = Robot;

    // 3. Настроить отображение
    SetOutline(SelectedRobot, true);

    UPathFollowerComponent* PathFollower = SelectedRobot->FindComponentByClass<UPathFollowerComponent>();
    if (PathFollower)
    {
        PathFollower->bDrawDebugPath = true;
        PathFollower->DebugDrawDuration = 999.0f;
    }

    // 4. Показать панель
    ShowRobotInfo(SelectedRobot);

    // 5. Запустить таймер обновления
    GetWorldTimerManager().SetTimer(
        InfoUpdateTimer,
        this,
        &AWarehouseGameMode::UpdateRobotInfo,
        0.5f,
        true
    );
}


void AWarehouseGameMode::DeselectRobot()
{
    if (!SelectedRobot) return;

    // Сначала используем SelectedRobot
    SetOutline(SelectedRobot, false);

    UPathFollowerComponent* PathFollower = SelectedRobot->FindComponentByClass<UPathFollowerComponent>();
    if (PathFollower)
    {
        PathFollower->bDrawDebugPath = false;
        PathFollower->ClearDebugPath();
    }

    HideRobotInfo();

    // Обнуляем в конце
    SelectedRobot = nullptr;

    GetWorldTimerManager().ClearTimer(InfoUpdateTimer);
}

void AWarehouseGameMode::ShowRobotInfo(AActor* Robot)
{
    if (!RobotInfoWidgetClass || !Robot) return;

    RobotInfoWidget = CreateWidget<UUserWidget>(GetWorld(), RobotInfoWidgetClass);
    if (RobotInfoWidget)
    {
        RobotInfoWidget->AddToViewport();
        UpdateRobotInfo();
    }
}

void AWarehouseGameMode::HideRobotInfo()
{
    if (RobotInfoWidget)
    {
        RobotInfoWidget->RemoveFromParent();
        RobotInfoWidget = nullptr;
    }
}

void AWarehouseGameMode::UpdateRobotInfo()
{
    if (!RobotInfoWidget || !SelectedRobot) return;

    // Имя робота
    UTextBlock* NameText = Cast<UTextBlock>(
        RobotInfoWidget->GetWidgetFromName(FName(TEXT("RobotNameText")))
    );
    if (NameText)
    {
        NameText->SetText(FText::FromString(SelectedRobot->GetName()));
    }

    // Статус
    UTextBlock* StatusText = Cast<UTextBlock>(
        RobotInfoWidget->GetWidgetFromName(FName(TEXT("RobotStatusText")))
    );
    if (StatusText)
    {
        StatusText->SetText(FText::FromString(TEXT("Active")));
    }

    // Скорость
    UTextBlock* SpeedText = Cast<UTextBlock>(
        RobotInfoWidget->GetWidgetFromName(FName(TEXT("RobotSpeedText")))
    );
    if (SpeedText)
    {
        ACharacter* Char = Cast<ACharacter>(SelectedRobot);
        if (Char && Char->GetCharacterMovement())
        {
            float Speed = Char->GetCharacterMovement()->Velocity.Size();
            SpeedText->SetText(FText::FromString(
                FString::Printf(TEXT("%.0f cm/s"), Speed)
            ));
        }
    }

    // Маршрут
    UTextBlock* PathText = Cast<UTextBlock>(
        RobotInfoWidget->GetWidgetFromName(FName(TEXT("PathInfoText")))
    );
    if (PathText)
    {
        UPathFollowerComponent* PF = SelectedRobot->FindComponentByClass<UPathFollowerComponent>();
        if (PF)
        {
            PathText->SetText(FText::FromString(
                FString::Printf(TEXT("Point %d/%d"), PF->CurrentIndex, PF->Waypoints.Num())
            ));
        }
    }
}

void AWarehouseGameMode::ExportSimulationData()
{
    USimulationDataCollector* Collector = USimulationDataCollector::GetCollector(this);
    if (!Collector)
    {
        UE_LOG(LogTemp, Error, TEXT("SimulationDataCollector not found!"));
        return;
    }

    FString FileName = FString::Printf(TEXT("SimData_%s.csv"),
        *FDateTime::Now().ToString(TEXT("%Y%m%d_%H%M%S")));
    FString FilePath = FPaths::ProjectSavedDir() + FileName;

    Collector->ExportToCSV(FilePath);
    UE_LOG(LogTemp, Log, TEXT("Data exported to: %s"), *FilePath);
}

void AWarehouseGameMode::SetOutline(AActor* Actor, bool bEnabled)
{
    if (!Actor) return;

    // Найти все меши и включить/выключить Custom Depth
    TArray<UMeshComponent*> Meshes;
    Actor->GetComponents<UMeshComponent>(Meshes);

    for (UMeshComponent* Mesh : Meshes)
    {
        Mesh->SetRenderCustomDepth(bEnabled);
        Mesh->SetCustomDepthStencilValue(bEnabled ? 1 : 0);
    }


}

void AWarehouseGameMode::SetSimulationSpeed(float Speed)
{
    CustomTimeDilation = FMath::Clamp(Speed, 0.1f, 10.0f);

    if (!bSimulationPaused)
    {
        UGameplayStatics::SetGlobalTimeDilation(GetWorld(), CustomTimeDilation);
    }

    UE_LOG(LogTemp, Log, TEXT("Simulation speed: %.1fx"), CustomTimeDilation);
}

void AWarehouseGameMode::PauseSimulation()
{
    bSimulationPaused = true;
    UGameplayStatics::SetGlobalTimeDilation(GetWorld(), 0.0f);

    UE_LOG(LogTemp, Log, TEXT("Simulation PAUSED"));
}

void AWarehouseGameMode::ResumeSimulation()
{
    bSimulationPaused = false;
    UGameplayStatics::SetGlobalTimeDilation(GetWorld(), CustomTimeDilation);

    UE_LOG(LogTemp, Log, TEXT("Simulation RESUMED at %.1fx"), CustomTimeDilation);
}

void AWarehouseGameMode::ToggleMAPFDebug()
{
    UMAPFPlanner* Planner = UMAPFPlanner::GetPlanner(this);
    if (Planner)
    {
        Planner->ToggleReservationVisualization();
    }
}