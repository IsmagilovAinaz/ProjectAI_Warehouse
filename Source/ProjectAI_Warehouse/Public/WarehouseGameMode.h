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

private:
    // Обработчик клика
    UFUNCTION()
    void OnRobotClicked(AActor* ClickedActor, FKey Button);

    // Подписаться на клики всех роботов
    void BindRobotClicks();
};