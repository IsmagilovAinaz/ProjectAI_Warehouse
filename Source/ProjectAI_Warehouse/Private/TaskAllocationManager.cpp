#include "TaskAllocationManager.h"
#include "TaskAuctionManager.h"
#include "RobotAuctionComponent.h"
#include "Kismet/GameplayStatics.h"
#include "Engine/World.h"

UTaskAllocationManager::UTaskAllocationManager()
{
    PrimaryComponentTick.bCanEverTick = false;
}

UTaskAllocationManager* UTaskAllocationManager::GetManager(UObject* WorldContext)
{
    if (!WorldContext) return nullptr;

    TArray<AActor*> Actors;
    UGameplayStatics::GetAllActorsOfClass(WorldContext->GetWorld(), AActor::StaticClass(), Actors);

    for (AActor* A : Actors)
    {
        UTaskAllocationManager* Mgr = A->FindComponentByClass<UTaskAllocationManager>();
        if (Mgr) return Mgr;
    }
    return nullptr;
}

void UTaskAllocationManager::SetStrategy(EAllocationStrategy NewStrategy)
{
    CurrentStrategy = NewStrategy;
    FString Name = (NewStrategy == EAllocationStrategy::Greedy) ? TEXT("Greedy") : TEXT("Auction");
    UE_LOG(LogTemp, Warning, TEXT("=== STRATEGY CHANGED: %s ==="), *Name);
}

ATaskAuctionManager* UTaskAllocationManager::FindAuctionManager() const
{
    TArray<AActor*> Actors;
    UGameplayStatics::GetAllActorsOfClass(GetWorld(), ATaskAuctionManager::StaticClass(), Actors);
    return Actors.Num() > 0 ? Cast<ATaskAuctionManager>(Actors[0]) : nullptr;
}

FAuctionResult UTaskAllocationManager::AllocateTask(
    const FString& TaskID,
    FVector TaskLocation,
    float TaskPriority,
    float PayloadMass,
    const TMap<int32, float>& EstimatedTravelTimes)
{
    UE_LOG(LogTemp, Log, TEXT("Allocation: Task %s using %s strategy"),
        *TaskID,
        CurrentStrategy == EAllocationStrategy::Greedy ? TEXT("Greedy") : TEXT("Auction"));

    switch (CurrentStrategy)
    {
    case EAllocationStrategy::Greedy:
        return GreedyAllocation(TaskID, TaskLocation, PayloadMass);

    case EAllocationStrategy::Auction:
        return AuctionAllocation(TaskID, TaskLocation, TaskPriority, PayloadMass, EstimatedTravelTimes);

    default:
        FAuctionResult FailResult;
        FailResult.TaskID = TaskID;
        FailResult.bSuccess = false;
        return FailResult;
    }
}

FAuctionResult UTaskAllocationManager::GreedyAllocation(
    const FString& TaskID,
    FVector TaskLocation,
    float PayloadMass) const
{
    FAuctionResult Result;
    Result.TaskID = TaskID;
    Result.bSuccess = false;

    ATaskAuctionManager* AuctionMgr = FindAuctionManager();
    if (!AuctionMgr)
    {
        UE_LOG(LogTemp, Error, TEXT("Greedy: AuctionManager not found!"));
        return Result;
    }

    TArray<URobotAuctionComponent*> Robots = AuctionMgr->GetFeasibleRobots(PayloadMass);

    if (Robots.Num() == 0)
    {
        UE_LOG(LogTemp, Error, TEXT("Greedy: No feasible robots!"));
        return Result;
    }

    URobotAuctionComponent* BestRobot = nullptr;
    float MinDistance = FLT_MAX;

    for (URobotAuctionComponent* Robot : Robots)
    {
        float Dist = FVector::Dist(Robot->GetCurrentLocation(), TaskLocation);

        // Создаём ставку для логирования
        FAuctionBid Bid;
        Bid.RobotID = Robot->RobotID;
        Bid.BidValue = Dist;
        Bid.TravelTime = Dist / 300.0f; // Примерная скорость
        Result.AllBids.Add(Bid);

        UE_LOG(LogTemp, Log, TEXT("  Greedy: Robot %d distance = %.0f"), Robot->RobotID, Dist);

        if (Dist < MinDistance)
        {
            MinDistance = Dist;
            BestRobot = Robot;
        }
    }

    if (BestRobot)
    {
        Result.WinnerRobotID = BestRobot->RobotID;
        Result.WinningBid = MinDistance;
        Result.bSuccess = true;

        UE_LOG(LogTemp, Warning, TEXT("Greedy WINNER: Robot %d (distance %.0f)"),
            BestRobot->RobotID, MinDistance);
    }

    return Result;
}

FAuctionResult UTaskAllocationManager::AuctionAllocation(
    const FString& TaskID,
    FVector TaskLocation,
    float TaskPriority,
    float PayloadMass,
    const TMap<int32, float>& EstimatedTravelTimes)
{
    ATaskAuctionManager* AuctionMgr = FindAuctionManager();
    if (!AuctionMgr)
    {
        FAuctionResult FailResult;
        FailResult.TaskID = TaskID;
        FailResult.bSuccess = false;
        return FailResult;
    }

    return AuctionMgr->RunAuction(TaskID, TaskLocation, TaskPriority, PayloadMass, EstimatedTravelTimes);
}