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

    // Получаем цель и ID агента
    FVector Goal = BB->GetValueAsVector(GoalLocationKey);
    CurrentAgentID = BB->GetValueAsInt(AgentIDKey);

    if (Goal.IsNearlyZero() || CurrentAgentID < 0)
    {
        UE_LOG(LogTemp, Error, TEXT("BTTask: Invalid goal or AgentID"));
        return EBTNodeResult::Failed;
    }

    // Получаем персонажа
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

    // Получаем компонент следования
    UPathFollowerComponent* Follower = Pawn->FindComponentByClass<UPathFollowerComponent>();
    if (!Follower)
    {
        UE_LOG(LogTemp, Error, TEXT("BTTask: PathFollowerComponent not found on pawn"));
        return EBTNodeResult::Failed;
    }

    // Получаем планировщик
    UMAPFPlanner* Planner = UMAPFPlanner::GetPlanner(Pawn);
    if (!Planner)
    {
        UE_LOG(LogTemp, Error, TEXT("BTTask: MAPFPlanner subsystem not found"));
        return EBTNodeResult::Failed;
    }

    // Планируем путь
    float CurrentTime = Pawn->GetWorld()->GetTimeSeconds();
    FVector StartPos = Pawn->GetActorLocation();

    UE_LOG(LogTemp, Log, TEXT("BTTask: Planning path for Agent %d from (%.1f,%.1f) to (%.1f,%.1f)"),
        CurrentAgentID, StartPos.X, StartPos.Y, Goal.X, Goal.Y);

    TArray<FVector> Path = Planner->PlanPath(CurrentAgentID, StartPos, Goal, CurrentTime);

    if (Path.Num() < 2)
    {
        UE_LOG(LogTemp, Warning, TEXT("BTTask: No valid path found for Agent %d"), CurrentAgentID);
        return EBTNodeResult::Failed;
    }

    // Конвертируем в сетку для резервирования
    TArray<FIntVector> GridPath;
    GridPath.Reserve(Path.Num());
    for (const FVector& P : Path)
    {
        GridPath.Add(Planner->WorldToGrid(P));
    }

    // Резервируем путь
    if (!Planner->ReservePath(CurrentAgentID, GridPath))
    {
        UE_LOG(LogTemp, Warning, TEXT("BTTask: Failed to reserve path for Agent %d"), CurrentAgentID);
        return EBTNodeResult::Failed;
    }

    // Передаём путь компоненту движения
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

    // Таймаут планирования
    if (ElapsedTime >= PlanningTimeout)
    {
        UE_LOG(LogTemp, Warning, TEXT("BTTask: Planning timeout for Agent %d"), CurrentAgentID);
        FinishLatentTask(OwnerComp, EBTNodeResult::Failed);
        bIsRunning = false;
        return;
    }

    // Проверяем завершение движения
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