#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "WarehouseGameMode.generated.h"

UCLASS()
class PROJECTAI_WAREHOUSE_API AWarehouseGameMode : public AGameModeBase
{
    GENERATED_BODY()

public:
    AWarehouseGameMode();

    virtual void BeginPlay() override;

    // Виджет HUD
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UI")
    TSubclassOf<class UUserWidget> HUDWidgetClass;

    UPROPERTY(BlueprintReadOnly, Category = "UI")
    class UUserWidget* HUDWidget;

    UPROPERTY(EditAnywhere, Category = "UI")
    TSubclassOf<UUserWidget> RobotInfoWidgetClass;

    UPROPERTY()
    UUserWidget* RobotInfoWidget;

    // Выбранный робот
    UPROPERTY(BlueprintReadOnly, Category = "Selection")
    AActor* SelectedRobot;

    // Выбрать/снять робота
    UFUNCTION(BlueprintCallable, Category = "Selection")
    void SelectRobot(AActor* Robot);

    UFUNCTION(BlueprintCallable, Category = "Selection")
    void DeselectRobot();

    // Экспорт данных
    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void ExportSimulationData();

    void ShowRobotInfo(AActor* Robot);
    void HideRobotInfo();
    void UpdateRobotInfo();

    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void SetSimulationSpeed(float Speed);

    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void PauseSimulation();

    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void ResumeSimulation();

    UFUNCTION(BlueprintCallable, Category = "Simulation")
    bool IsSimulationPaused() const { return bSimulationPaused; }

    UFUNCTION(BlueprintCallable, Category = "Simulation")
    float GetSimulationSpeed() const { return CustomTimeDilation; }

    UFUNCTION(Exec)
    void ToggleMAPFDebug();


private:
    bool bSimulationPaused = false;
    float CustomTimeDilation = 1.0f;

    // Обработчик клика
    UFUNCTION()
    void OnRobotClicked(AActor* ClickedActor, FKey Button);

    // Подписаться на клики всех роботов
    void BindRobotClicks();

    void SetOutline(AActor* Actor, bool bEnabled);

    FTimerHandle InfoUpdateTimer;
};