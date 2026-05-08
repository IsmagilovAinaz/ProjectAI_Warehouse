#include "WarehouseGameMode.h"
#include "SimulationDataCollector.h"
#include "Kismet/GameplayStatics.h"
#include "Blueprint/UserWidget.h"
#include "Engine/World.h"
#include "Misc/DateTime.h"
#include "Misc/Paths.h"
#include "HAL/PlatformFilemanager.h"
#include "ClickableRobotInterface.h"

AWarehouseGameMode::AWarehouseGameMode()
{
    // Используем Spectator Pawn по умолчанию
    DefaultPawnClass = nullptr; // Будет использован SpectatorPawn из движка
}

void AWarehouseGameMode::BeginPlay()
{
    Super::BeginPlay();

    // Создать HUD
    if (HUDWidgetClass)
    {
        HUDWidget = CreateWidget<UUserWidget>(GetWorld(), HUDWidgetClass);
        if (HUDWidget)
        {
            HUDWidget->AddToViewport();
        }
    }

    // Подписаться на клики роботов
    BindRobotClicks();

    UE_LOG(LogTemp, Log, TEXT("WarehouseGameMode: Initialized. Click on robots to select them."));
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
    if (!Robot) return;

    // Снять выделение с предыдущего
    DeselectRobot();

    SelectedRobot = Robot;
    UE_LOG(LogTemp, Log, TEXT("Selected: %s"), *Robot->GetName());
}

void AWarehouseGameMode::DeselectRobot()
{
    if (!SelectedRobot) return;

    UE_LOG(LogTemp, Log, TEXT("Deselected: %s"), *SelectedRobot->GetName());
    SelectedRobot = nullptr;
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