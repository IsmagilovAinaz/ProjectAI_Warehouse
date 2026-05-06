#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "AuctionTypes.h"
#include "RobotAuctionComponent.generated.h"

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class PROJECTAI_WAREHOUSE_API URobotAuctionComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    URobotAuctionComponent();

    // === КОНФИГУРАЦИЯ ===
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Auction|Config")
    FAuctionWeights Weights;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Auction|Config")
    int32 RobotID = -1;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Auction|Config")
    float MaxBatteryLevel = 100.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Auction|Config")
    float MinBatteryForTask = 10.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Auction|Config")
    float MaxPayloadCapacity = 1000.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Auction|Config")
    int32 MaxQueueSize = 5;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Auction|Config")
    float AverageSpeed = 200.0f;  // см/с - средняя скорость робота

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Auction|Config")
    float AverageTaskDuration = 10.0f;  // секунд на задачу в очереди

    // === ТЕКУЩЕЕ СОСТОЯНИЕ ===
    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Auction|State")
    float CurrentBatteryLevel = 100.0f;

    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Auction|State")
    int32 CurrentQueueSize = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Auction|State")
    FString CurrentState = TEXT("idle");

    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Auction|State")
    FVector CurrentLocation;

    // === ОЧЕРЕДЬ ЗАДАЧ ===
    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Auction|Tasks")
    TArray<FString> TaskQueue;

    // === МЕТОДЫ АУКЦИОНА (ДОСТУПНЫ В BLUEPRINT) ===

    /**
     * Рассчитать ставку на задачу
     * @param TaskLocation - позиция задачи в мире
     * @param TaskPriority - приоритет задачи (>1 = срочная)
     * @param PayloadMass - масса груза (кг)
     * @param EstimatedTravelTime - ОЦЕНКА времени в пути (вычисляется вызывающим кодом)
     * @return ставка робота
     */
    UFUNCTION(BlueprintCallable, Category = "Auction")
    FAuctionBid CalculateBid(
        const FString& TaskID,
        FVector TaskLocation,
        float TaskPriority,
        float PayloadMass,
        float EstimatedTravelTime
    );

    /**
     * Быстрая оценка времени в пути (без MAPF)
     * Использует евклидово расстояние / среднюю скорость
     */
    UFUNCTION(BlueprintCallable, Category = "Auction")
    float EstimateTravelTime(FVector StartLocation, FVector EndLocation) const;

    /**
     * Проверка, может ли робот выполнить задачу
     */
    UFUNCTION(BlueprintCallable, Category = "Auction")
    bool IsFeasible(float PayloadMass) const;

    UFUNCTION(BlueprintCallable, Category = "Auction")
    void AddTaskToQueue(const FString& TaskID);

    UFUNCTION(BlueprintCallable, Category = "Auction")
    void RemoveTaskFromQueue(const FString& TaskID);

    UFUNCTION(BlueprintPure, Category = "Auction")
    float GetNormalizedBattery() const { return CurrentBatteryLevel / FMath::Max(1.0f, MaxBatteryLevel); }

    UFUNCTION(BlueprintPure, Category = "Auction")
    float GetQueueLoad() const { return (float)CurrentQueueSize / FMath::Max(1, MaxQueueSize); }

    UFUNCTION(BlueprintPure, Category = "Auction")
    FVector GetCurrentLocation() const;

private:
    float NormalizeValue(float Value, float MaxValue) const;
};