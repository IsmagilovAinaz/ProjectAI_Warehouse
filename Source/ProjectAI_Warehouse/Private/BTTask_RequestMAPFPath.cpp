#include "BTTask_RequestMAPFPath.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "AIController.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/Character.h"
#include "MAPFPlanner.h"
#include "PathFollowerComponent.h"

UBTTask_RequestMAPFPath::UBTTask_RequestMAPFPath()
{
    NodeName = TEXT("Request MAPF Path");
    bNotifyTick = true;
    bNotifyTaskFinished = true;
}

EBTNodeResult::Type UBTTask_RequestMAPFPath::ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{
    UBlackboardComponent* BB = OwnerComp.GetBlackboardComponent();
    if (!BB)
    {
        UE_LOG(LogTemp, Error, TEXT("BTTask: Blackboard not found"));
        return EBTNodeResult::Failed;
    }

    FVector Goal = BB->GetValueAsVector(GoalLocationKey);
    CurrentAgentID = BB->GetValueAsInt(AgentIDKey);

    // Get agent priority from blackboard (lower = higher priority)
    float AgentPriority = BB->GetValueAsFloat("AgentPriority");
    if (AgentPriority <= 0.0f)
    {
        AgentPriority = static_cast<float>(CurrentAgentID) * 0.1f; // Default: ID-based priority
    }

    if (Goal.IsNearlyZero() || CurrentAgentID < 0)
    {
        UE_LOG(LogTemp, Error, TEXT("BTTask: Invalid goal or AgentID"));
        return EBTNodeResult::Failed;
    }

    AAIController* AIController = OwnerComp.GetAIOwner();
    if (!AIController) return EBTNodeResult::Failed;

    APawn* Pawn = AIController->GetPawn();
    if (!Pawn) return EBTNodeResult::Failed;

    ACharacter* Char = Cast<ACharacter>(Pawn);
    if (!Char)
    {
        UE_LOG(LogTemp, Warning, TEXT("BTTask: Pawn is not a Character"));
        return EBTNodeResult::Failed;
    }

    UPathFollowerComponent* Follower = Pawn->FindComponentByClass<UPathFollowerComponent>();
    if (!Follower)
    {
        UE_LOG(LogTemp, Error, TEXT("BTTask: PathFollowerComponent not found on pawn"));
        return EBTNodeResult::Failed;
    }

    UMAPFPlanner* Planner = UMAPFPlanner::GetPlanner(Pawn);
    if (!Planner)
    {
        UE_LOG(LogTemp, Error, TEXT("BTTask: MAPFPlanner subsystem not found"));
        return EBTNodeResult::Failed;
    }

    // Plan path with priority
    float CurrentTime = Pawn->GetWorld()->GetTimeSeconds();
    FVector StartPos = Pawn->GetActorLocation();

    UE_LOG(LogTemp, Log, TEXT("BTTask: Planning path for Agent %d (priority %.2f) from (%.1f,%.1f) to (%.1f,%.1f)"),
        CurrentAgentID, AgentPriority, StartPos.X, StartPos.Y, Goal.X, Goal.Y);

    TArray<FVector> Path = Planner->PlanPath(CurrentAgentID, StartPos, Goal, CurrentTime, AgentPriority);

    if (Path.Num() < 2)
    {
        UE_LOG(LogTemp, Warning, TEXT("BTTask: No valid path found for Agent %d"), CurrentAgentID);
        return EBTNodeResult::Failed;
    }

    // Convert to grid for reservation
    TArray<FIntVector> GridPath;
    GridPath.Reserve(Path.Num());
    for (const FVector& P : Path)
    {
        GridPath.Add(Planner->WorldToGrid(P));
    }

    // Try to reserve path
    if (!Planner->ReservePath(CurrentAgentID, GridPath, AgentPriority))
    {
        UE_LOG(LogTemp, Warning, TEXT("BTTask: Failed to reserve path for Agent %d - replanning with higher priority"), CurrentAgentID);

        // Try again with higher priority
        Path = Planner->PlanPath(CurrentAgentID, StartPos, Goal, CurrentTime, -1.0f);
        if (Path.Num() < 2)
        {
            return EBTNodeResult::Failed;
        }

        GridPath.Empty();
        for (const FVector& P : Path)
        {
            GridPath.Add(Planner->WorldToGrid(P));
        }

        if (!Planner->ReservePath(CurrentAgentID, GridPath, -1.0f))
        {
            return EBTNodeResult::Failed;
        }
    }

    // Set path to follower
    Follower->AgentID = CurrentAgentID;
    Follower->SetPath(Path);

    UE_LOG(LogTemp, Log, TEXT("BTTask: Path reserved and set for Agent %d (%d points)"),
        CurrentAgentID, Path.Num());

    bIsRunning = true;
    ElapsedTime = 0.0f;

    return EBTNodeResult::InProgress;
}

void UBTTask_RequestMAPFPath::TickTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds)
{
    if (!bIsRunning) return;

    ElapsedTime += DeltaSeconds;

    // Timeout check
    if (ElapsedTime >= PlanningTimeout)
    {
        UE_LOG(LogTemp, Warning, TEXT("BTTask: Planning timeout for Agent %d"), CurrentAgentID);
        FinishLatentTask(OwnerComp, EBTNodeResult::Failed);
        bIsRunning = false;
        return;
    }

    // Check path completion
    AAIController* AIController = OwnerComp.GetAIOwner();
    if (!AIController) return;

    APawn* Pawn = AIController->GetPawn();
    if (!Pawn) return;

    UPathFollowerComponent* Follower = Pawn->FindComponentByClass<UPathFollowerComponent>();
    if (Follower && Follower->IsPathComplete())
    {
        UE_LOG(LogTemp, Log, TEXT("BTTask: Path completed for Agent %d"), CurrentAgentID);
        FinishLatentTask(OwnerComp, EBTNodeResult::Succeeded);
        bIsRunning = false;
    }
}