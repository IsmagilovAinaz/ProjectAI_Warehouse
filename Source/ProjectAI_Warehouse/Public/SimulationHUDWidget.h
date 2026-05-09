#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "SimulationHUDWidget.generated.h"

UCLASS()
class PROJECTAI_WAREHOUSE_API USimulationHUDWidget : public UUserWidget
{
    GENERATED_BODY()

public:
    UPROPERTY()
    TObjectPtr<class UButton> StrategyGreedyBtn;

    UPROPERTY()
    TObjectPtr<class UButton> StrategyAuctionBtn;

    virtual void NativeConstruct() override;
    virtual void NativeTick(const FGeometry& MyGeometry, float InDeltaTime) override;

    // Обновление всех метрик
    void UpdateStats();

    // Экспорт CSV
    UFUNCTION(BlueprintCallable)
    void ExportCSV();

    UFUNCTION()
    void OnStrategyAuctionClicked();

    UFUNCTION()
    void OnStrategyGreedyClicked();


    // Стратегия аукциона
    void SetStrategyText(const FString& StrategyName);

    UPROPERTY(BlueprintReadWrite, Category = "Stats")
    TObjectPtr<class AActor> TaskManager;

    // Переменные для статистики
    UPROPERTY(BlueprintReadOnly, Category = "Stats")
    int32 ActiveTaskCount = 0;

    UPROPERTY(BlueprintReadOnly, Category = "Stats")
    int32 CompletedTaskCount = 0;

    UPROPERTY()
    TObjectPtr<class UButton> PauseBtn;

    UPROPERTY()
    TObjectPtr<class UButton> ResumeBtn;

    UPROPERTY()
    TObjectPtr<class UButton> Speed1xBtn;

    UPROPERTY()
    TObjectPtr<class UButton> Speed2xBtn;

    UPROPERTY()
    TObjectPtr<class UButton> Speed5xBtn;

    UPROPERTY()
    TObjectPtr<class UButton> Speed10xBtn;

    UPROPERTY()
    TObjectPtr<class UButton> ExportBtn;

    // Обработчики
    UFUNCTION()
    void OnPauseClicked();

    UFUNCTION()
    void OnResumeClicked();

    UFUNCTION()
    void OnSpeed1xClicked();

    UFUNCTION()
    void OnSpeed2xClicked();

    UFUNCTION()
    void OnSpeed5xClicked();

    UFUNCTION()
    void OnSpeed10xClicked();

    UPROPERTY()
    TObjectPtr<class UButton> MapfDebugBtn;

    UFUNCTION()
    void OnMapfDebugClicked();

protected:
    // Виджеты (имена должны совпадать с именами в Blueprint)
    UPROPERTY(meta = (BindWidget))
    class UTextBlock* RobotCountText;

    UPROPERTY(meta = (BindWidget))
    class UTextBlock* ActiveTaskCountText;

    UPROPERTY(meta = (BindWidget))
    class UTextBlock* CompletedTaskCountText;

    UPROPERTY(meta = (BindWidget))
    class UTextBlock* ConflictCountText;

    UPROPERTY(meta = (BindWidget))
    class UTextBlock* ReplanCountText;

    UPROPERTY(meta = (BindWidget))
    class UTextBlock* AvgTaskTimeText;

    UPROPERTY(meta = (BindWidget))
    class UTextBlock* FPSText;

    UPROPERTY(meta = (BindWidget))
    class UTextBlock* SimulationTimeText;

    UPROPERTY(meta = (BindWidget))
    class UTextBlock* StrategyText;

    UPROPERTY(meta = (BindWidget))
    class UTextBlock* SpeedText;

private:
    class AWarehouseGameMode* GetGameMode() const;
    float SimulationStartTime;
    bool bSimulationRunning;

    UFUNCTION()
    void OnExportClicked();
};