#include "SimulationHUDWidget.h"
#include "SimulationDataCollector.h"
#include "PathFollowerComponent.h"
#include "Components/TextBlock.h"
#include "Components/Button.h"
#include "Kismet/GameplayStatics.h"
#include "Engine/World.h"
#include "Misc/DateTime.h"
#include "Misc/Paths.h"
#include "WarehouseGameMode.h"
#include "Blueprint/WidgetTree.h"
#include "MAPFPlanner.h"
#include "TaskAllocationManager.h"

void USimulationHUDWidget::NativeConstruct()
{
    Super::NativeConstruct();

    SimulationStartTime = GetWorld()->GetTimeSeconds();

    // Хелпер-лямбда для получения кнопки из WBP_Button
    auto GetButtonFromWidget = [this](const FName& WidgetName) -> UButton*
        {
            UUserWidget* Wrapper = Cast<UUserWidget>(GetWidgetFromName(WidgetName));
            if (!Wrapper)
            {
                UE_LOG(LogTemp, Warning, TEXT("HUD: Widget '%s' not found"), *WidgetName.ToString());
                return nullptr;
            }

            // Ищем кнопку внутри WBP_Button — проверьте имя в вашем WBP_Button!
            UButton* Btn = Cast<UButton>(Wrapper->GetWidgetFromName(TEXT("Button_0")));
            if (!Btn)
            {
                // Попробовать другие возможные имена
                Btn = Cast<UButton>(Wrapper->GetWidgetFromName(TEXT("MainButton")));
            }
            if (!Btn)
            {
                // Пройти по всем детям и найти первую UButton
                TArray<UWidget*> Children;
                Wrapper->WidgetTree->GetAllWidgets(Children);
                for (UWidget* Child : Children)
                {
                    Btn = Cast<UButton>(Child);
                    if (Btn) break;
                }
            }

            return Btn;
        };

    PauseBtn = GetButtonFromWidget(TEXT("PauseButton"));
    ResumeBtn = GetButtonFromWidget(TEXT("ResumeButton"));
    Speed1xBtn = GetButtonFromWidget(TEXT("Speed1xButton"));
    Speed2xBtn = GetButtonFromWidget(TEXT("Speed2xButton"));
    Speed5xBtn = GetButtonFromWidget(TEXT("Speed5xButton"));
    Speed10xBtn = GetButtonFromWidget(TEXT("Speed10xButton"));
    ExportBtn = Cast<UButton>(GetWidgetFromName(TEXT("ExportButton")));

    StrategyGreedyBtn = Cast<UButton>(GetWidgetFromName(TEXT("StrategyGreedyButton")));
    StrategyAuctionBtn = Cast<UButton>(GetWidgetFromName(TEXT("StrategyAuctionButton")));

    MapfDebugBtn = Cast<UButton>(GetWidgetFromName(TEXT("MapfDebugButton")));
    if (MapfDebugBtn)
        MapfDebugBtn->OnClicked.AddDynamic(this, &USimulationHUDWidget::OnMapfDebugClicked);

    // Привязать обработчики
    if (PauseBtn)
    {
        PauseBtn->OnClicked.AddDynamic(this, &USimulationHUDWidget::OnPauseClicked);
        UE_LOG(LogTemp, Log, TEXT("HUD: PauseButton OK"));
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("HUD: PauseButton FAILED"));
    }

    if (ResumeBtn)
        ResumeBtn->OnClicked.AddDynamic(this, &USimulationHUDWidget::OnResumeClicked);

    if (Speed1xBtn)
        Speed1xBtn->OnClicked.AddDynamic(this, &USimulationHUDWidget::OnSpeed1xClicked);

    if (Speed2xBtn)
        Speed2xBtn->OnClicked.AddDynamic(this, &USimulationHUDWidget::OnSpeed2xClicked);

    if (Speed5xBtn)
        Speed5xBtn->OnClicked.AddDynamic(this, &USimulationHUDWidget::OnSpeed5xClicked);

    if (Speed10xBtn)
        Speed10xBtn->OnClicked.AddDynamic(this, &USimulationHUDWidget::OnSpeed10xClicked);

    if (ExportBtn)
        ExportBtn->OnClicked.AddDynamic(this, &USimulationHUDWidget::OnExportClicked);

    if (StrategyGreedyBtn)
    {
        StrategyGreedyBtn->OnClicked.AddDynamic(this, &USimulationHUDWidget::OnStrategyGreedyClicked);
        UE_LOG(LogTemp, Log, TEXT("HUD: StrategyGreedyButton bound"));
    }

    if (StrategyAuctionBtn)
    {
        StrategyAuctionBtn->OnClicked.AddDynamic(this, &USimulationHUDWidget::OnStrategyAuctionClicked);
        UE_LOG(LogTemp, Log, TEXT("HUD: StrategyAuctionButton bound"));
    }

    UE_LOG(LogTemp, Warning, TEXT("HUD: PauseButton = %s"), PauseBtn ? TEXT("OK") : TEXT("NULL"));
    UE_LOG(LogTemp, Warning, TEXT("HUD: ResumeButton = %s"), ResumeBtn ? TEXT("OK") : TEXT("NULL"));
    UE_LOG(LogTemp, Warning, TEXT("HUD: Speed1xButton = %s"), Speed1xBtn ? TEXT("OK") : TEXT("NULL"));
    UE_LOG(LogTemp, Warning, TEXT("HUD: Speed2xButton = %s"), Speed2xBtn ? TEXT("OK") : TEXT("NULL"));
    UE_LOG(LogTemp, Warning, TEXT("HUD: Speed5xButton = %s"), Speed5xBtn ? TEXT("OK") : TEXT("NULL"));
    UE_LOG(LogTemp, Warning, TEXT("HUD: Speed10xButton = %s"), Speed10xBtn ? TEXT("OK") : TEXT("NULL"));
    UE_LOG(LogTemp, Warning, TEXT("HUD: ExportButton = %s"), ExportBtn ? TEXT("OK") : TEXT("NULL"));
    UE_LOG(LogTemp, Warning, TEXT("HUD: StrategyGreedyBtn = %s"), StrategyGreedyBtn ? TEXT("OK") : TEXT("NULL"));
    UE_LOG(LogTemp, Warning, TEXT("HUD: StrategyAuctionBtn = %s"), StrategyAuctionBtn ? TEXT("OK") : TEXT("NULL"));
}

void USimulationHUDWidget::NativeTick(const FGeometry& MyGeometry, float InDeltaTime)
{
    Super::NativeTick(MyGeometry, InDeltaTime);
    UpdateStats();
}

void USimulationHUDWidget::UpdateStats()
{
    USimulationDataCollector* Collector = USimulationDataCollector::GetCollector(this);

    // Считать роботов
    TArray<AActor*> Robots;
    UGameplayStatics::GetAllActorsOfClass(GetWorld(), AActor::StaticClass(), Robots);
    int32 RobotCount = 0;
    for (AActor* Actor : Robots)
    {
        if (Actor->FindComponentByClass<UPathFollowerComponent>())
        {
            RobotCount++;
        }
    }

    // Читаем данные из TaskManager
    if (TaskManager)
    {
        // Создаём PropertyFinder для чтения переменных Blueprint
        FProperty* ActiveProp = TaskManager->GetClass()->FindPropertyByName(TEXT("ActiveTasks"));
        FProperty* CompletedProp = TaskManager->GetClass()->FindPropertyByName(TEXT("CompletedTasks"));

        if (ActiveProp)
        {
            void* ValuePtr = ActiveProp->ContainerPtrToValuePtr<void>(TaskManager);
            if (ValuePtr)
            {
                FIntProperty* IntProp = CastField<FIntProperty>(ActiveProp);
                if (IntProp)
                {
                    ActiveTaskCount = IntProp->GetPropertyValue(ValuePtr);
                }
            }
        }

        if (CompletedProp)
        {
            void* ValuePtr = CompletedProp->ContainerPtrToValuePtr<void>(TaskManager);
            if (ValuePtr)
            {
                FIntProperty* IntProp = CastField<FIntProperty>(CompletedProp);
                if (IntProp)
                {
                    CompletedTaskCount = IntProp->GetPropertyValue(ValuePtr);
                }
            }
        }
    }

    // Обновить текстовые поля
    if (RobotCountText)
        RobotCountText->SetText(FText::FromString(FString::FromInt(RobotCount)));

    if (ActiveTaskCountText)
        ActiveTaskCountText->SetText(FText::FromString(FString::FromInt(ActiveTaskCount)));

    if (CompletedTaskCountText)
        CompletedTaskCountText->SetText(FText::FromString(FString::FromInt(CompletedTaskCount)));

    if (Collector)
    {
        if (ConflictCountText)
            ConflictCountText->SetText(FText::FromString(FString::FromInt(Collector->TotalConflictsDetected)));

        if (ReplanCountText)
            ReplanCountText->SetText(FText::FromString(FString::FromInt(Collector->TotalReplans)));
    }

    // FPS
    if (FPSText)
    {
        float DeltaTime = GetWorld()->GetDeltaSeconds();
        float FPS = DeltaTime > 0.0f ? 1.0f / DeltaTime : 0.0f;
        FPSText->SetText(FText::FromString(FString::Printf(TEXT("%.0f"), FPS)));
    }

    // Время симуляции
    if (SimulationTimeText)
    {
        float ElapsedTime = GetWorld()->GetTimeSeconds() - SimulationStartTime;
        int32 Minutes = FMath::FloorToInt(ElapsedTime / 60.0f);
        int32 Seconds = FMath::FloorToInt(ElapsedTime) % 60;
        SimulationTimeText->SetText(FText::FromString(
            FString::Printf(TEXT("%02d:%02d"), Minutes, Seconds)));
    }

    // Среднее время задачи (если есть данные)
    if (AvgTaskTimeText && CompletedTaskCount > 0 && SimulationStartTime > 0)
    {
        float AvgTime = (GetWorld()->GetTimeSeconds() - SimulationStartTime) / CompletedTaskCount;
        AvgTaskTimeText->SetText(FText::FromString(FString::Printf(TEXT("%.1f s"), AvgTime)));
    }

    if (SpeedText) // Добавьте BindWidget UTextBlock* SpeedText;
    {
        AWarehouseGameMode* GM = GetGameMode();
        if (GM)
        {
            if (GM->IsSimulationPaused())
                SpeedText->SetText(FText::FromString(TEXT("PAUSED")));
            else
                SpeedText->SetText(FText::FromString(
                    FString::Printf(TEXT("%.1fx"), GM->GetSimulationSpeed())));
        }
    }
}

void USimulationHUDWidget::SetStrategyText(const FString& StrategyName)
{
    if (StrategyText)
    {
        StrategyText->SetText(FText::FromString(StrategyName));
    }
}

void USimulationHUDWidget::OnExportClicked()
{
    ExportCSV();
}

void USimulationHUDWidget::ExportCSV()
{
    USimulationDataCollector* Collector = USimulationDataCollector::GetCollector(this);
    if (!Collector) return;

    FString FileName = FString::Printf(TEXT("SimData_%s.csv"),
        *FDateTime::Now().ToString(TEXT("%Y%m%d_%H%M%S")));
    FString FilePath = FPaths::ProjectSavedDir() + FileName;

    Collector->ExportToCSV(FilePath);

    UE_LOG(LogTemp, Log, TEXT("HUD: Data exported to %s"), *FilePath);
}


AWarehouseGameMode* USimulationHUDWidget::GetGameMode() const
{
    return Cast<AWarehouseGameMode>(UGameplayStatics::GetGameMode(GetWorld()));
}

void USimulationHUDWidget::OnPauseClicked()
{
    if (AWarehouseGameMode* GM = GetGameMode())
        GM->PauseSimulation();
}

void USimulationHUDWidget::OnResumeClicked()
{
    if (AWarehouseGameMode* GM = GetGameMode())
        GM->ResumeSimulation();
}

void USimulationHUDWidget::OnSpeed1xClicked()
{
    if (AWarehouseGameMode* GM = GetGameMode())
        GM->SetSimulationSpeed(1.0f);
}

void USimulationHUDWidget::OnSpeed2xClicked()
{
    if (AWarehouseGameMode* GM = GetGameMode())
        GM->SetSimulationSpeed(2.0f);
}

void USimulationHUDWidget::OnSpeed5xClicked()
{
    if (AWarehouseGameMode* GM = GetGameMode())
        GM->SetSimulationSpeed(5.0f);
}

void USimulationHUDWidget::OnSpeed10xClicked()
{
    if (AWarehouseGameMode* GM = GetGameMode())
        GM->SetSimulationSpeed(10.0f);
}

void USimulationHUDWidget::OnMapfDebugClicked()
{
    UMAPFPlanner* Planner = UMAPFPlanner::GetPlanner(this);
    if (Planner)
    {
        Planner->ToggleReservationVisualization();
    }
}


void USimulationHUDWidget::OnStrategyGreedyClicked()
{
    UTaskAllocationManager* Mgr = UTaskAllocationManager::GetManager(this);
    if (Mgr)
    {
        Mgr->SetStrategy(EAllocationStrategy::Greedy);
        UE_LOG(LogTemp, Log, TEXT("HUD: Strategy changed to Greedy"));
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("HUD: TaskAllocationManager not found!"));
    }
}

void USimulationHUDWidget::OnStrategyAuctionClicked()
{
    UTaskAllocationManager* Mgr = UTaskAllocationManager::GetManager(this);
    if (Mgr)
    {
        Mgr->SetStrategy(EAllocationStrategy::Auction);
        UE_LOG(LogTemp, Log, TEXT("HUD: Strategy changed to Auction"));
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("HUD: TaskAllocationManager not found!"));
    }
}