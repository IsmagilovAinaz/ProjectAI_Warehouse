#include "RobotAuctionComponent.h"
#include "GameFramework/Actor.h"

URobotAuctionComponent::URobotAuctionComponent()
{
    PrimaryComponentTick.bCanEverTick = false;
}

float URobotAuctionComponent::EstimateTravelTime(FVector StartLocation, FVector EndLocation) const
{
    // Быстрая оценка: расстояние / скорость
    float Distance = FVector::Dist(StartLocation, EndLocation);
    float TravelTime = Distance / FMath::Max(1.0f, AverageSpeed);

    // Добавляем время на маневры (повороты, ускорение/торможение)
    float ManeuverTime = 2.0f; // 2 секунды на старт и остановку
    TravelTime += ManeuverTime;

    // Учёт очереди: каждая задача в очереди добавляет задержку
    float QueueDelay = CurrentQueueSize * AverageTaskDuration;
    TravelTime += QueueDelay;

    return TravelTime;
}

FAuctionBid URobotAuctionComponent::CalculateBid(
    const FString& TaskID,
    FVector TaskLocation,
    float TaskPriority,
    float PayloadMass,
    float EstimatedTravelTime)
{
    FAuctionBid Bid;
    Bid.RobotID = RobotID;
    Bid.TaskID = TaskID;

    // Проверка feasibility
    if (!IsFeasible(PayloadMass))
    {
        Bid.BidValue = FLT_MAX;
        Bid.CapacityPenalty = 1.0f;
        return Bid;
    }

    // 1. Время в пути с учётом очереди
    float QueueDelay = CurrentQueueSize * AverageTaskDuration;
    Bid.TravelTime = EstimatedTravelTime + QueueDelay;

    // 2. Штраф за батарею
    float NormalizedBattery = GetNormalizedBattery();
    Bid.BatteryPenalty = 1.0f - NormalizedBattery;

    // 3. Штраф за загруженность
    Bid.QueuePenalty = GetQueueLoad();

    // 4. Штраф за грузоподъёмность
    float CapacityRatio = PayloadMass / FMath::Max(1.0f, MaxPayloadCapacity);
    Bid.CapacityPenalty = FMath::Clamp(CapacityRatio, 0.0f, 1.0f);

    // Нормализация времени в пути
    float MaxExpectedTravel = 120.0f;
    float NormalizedTravel = FMath::Clamp(Bid.TravelTime / MaxExpectedTravel, 0.0f, 1.0f);

    // Итоговая ставка по формуле (6)
    Bid.BidValue =
        Weights.TravelWeight * NormalizedTravel +
        Weights.BatteryWeight * Bid.BatteryPenalty +
        Weights.QueueWeight * Bid.QueuePenalty +
        Weights.CapacityWeight * Bid.CapacityPenalty;

    // Применение срочности задачи (формула 8)
    Bid.UrgencyMultiplier = FMath::Max(1.0f, TaskPriority);
    Bid.BidValue /= Bid.UrgencyMultiplier;

    Bid.BidValue = FMath::Clamp(Bid.BidValue, 0.0f, 1.0f);

    return Bid;
}

bool URobotAuctionComponent::IsFeasible(float PayloadMass) const
{
    if (CurrentBatteryLevel < MinBatteryForTask) return false;
    if (PayloadMass > MaxPayloadCapacity) return false;
    if (CurrentState == TEXT("fault")) return false;
    if (CurrentQueueSize >= MaxQueueSize) return false;
    return true;
}

void URobotAuctionComponent::AddTaskToQueue(const FString& TaskID)
{
    if (!TaskQueue.Contains(TaskID))
    {
        TaskQueue.Add(TaskID);
        CurrentQueueSize = TaskQueue.Num();

        UE_LOG(LogTemp, Log, TEXT("Auction: Robot %d added task %s (queue: %d)"),
            RobotID, *TaskID, CurrentQueueSize);
    }
}

void URobotAuctionComponent::RemoveTaskFromQueue(const FString& TaskID)
{
    TaskQueue.Remove(TaskID);
    CurrentQueueSize = TaskQueue.Num();

    UE_LOG(LogTemp, Log, TEXT("Auction: Robot %d removed task %s (queue: %d)"),
        RobotID, *TaskID, CurrentQueueSize);
}

FVector URobotAuctionComponent::GetCurrentLocation() const
{
    if (AActor* Owner = GetOwner())
    {
        return Owner->GetActorLocation();
    }
    return CurrentLocation;
}

float URobotAuctionComponent::NormalizeValue(float Value, float MaxValue) const
{
    if (MaxValue <= 0.0f) return 0.0f;
    return FMath::Clamp(Value / MaxValue, 0.0f, 1.0f);
}