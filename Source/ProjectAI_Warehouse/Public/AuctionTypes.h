#pragma once

#include "CoreMinimal.h"
#include "AuctionTypes.generated.h"

// Аукционная ставка робота на задачу
USTRUCT(BlueprintType)
struct FAuctionBid
{
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly)
    int32 RobotID;

    UPROPERTY(BlueprintReadOnly)
    FString TaskID;

    UPROPERTY(BlueprintReadOnly)
    float BidValue;  // Итоговая ставка (чем меньше, тем лучше)

    UPROPERTY(BlueprintReadOnly)
    float TravelTime;  // Время в пути

    UPROPERTY(BlueprintReadOnly)
    float BatteryPenalty;  // Штраф за низкий заряд

    UPROPERTY(BlueprintReadOnly)
    float QueuePenalty;  // Штраф за загруженность

    UPROPERTY(BlueprintReadOnly)
    float CapacityPenalty;  // Штраф за превышение грузоподъёмности

    UPROPERTY(BlueprintReadOnly)
    float UrgencyMultiplier;  // Множитель срочности задачи

    FAuctionBid()
        : RobotID(-1), BidValue(FLT_MAX), TravelTime(0), BatteryPenalty(0),
        QueuePenalty(0), CapacityPenalty(0), UrgencyMultiplier(1.0f) {
    }
};

// Результат аукциона
USTRUCT(BlueprintType)
struct FAuctionResult
{
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly)
    FString TaskID;

    UPROPERTY(BlueprintReadOnly)
    int32 WinnerRobotID;

    UPROPERTY(BlueprintReadOnly)
    float WinningBid;

    UPROPERTY(BlueprintReadOnly)
    TArray<FAuctionBid> AllBids;  // Все ставки для анализа

    UPROPERTY(BlueprintReadOnly)
    bool bSuccess;

    FAuctionResult() : WinnerRobotID(-1), WinningBid(FLT_MAX), bSuccess(false) {}
};

// Весовые коэффициенты для расчёта ставки
USTRUCT(BlueprintType)
struct FAuctionWeights
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float TravelWeight = 0.35f;  // ?1 - вес времени в пути

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float BatteryWeight = 0.25f;  // ?2 - вес уровня заряда

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float QueueWeight = 0.25f;  // ?3 - вес загруженности

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float CapacityWeight = 0.15f;  // ?4 - вес грузоподъёмности

    bool IsValid() const
    {
        return FMath::IsNearlyEqual(TravelWeight + BatteryWeight + QueueWeight + CapacityWeight, 1.0f, 0.01f);
    }
};