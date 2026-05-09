#include "BTTask_RequestMAPFPath.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "AIController.h"
#include "GameFramework/Pawn.h"
#include "MAPFPlanner.h"
#include "PathFollowerComponent.h"

UBTTask_RequestMAPFPath::UBTTask_RequestMAPFPath()
{
    NodeName = TEXT("Request MAPF Path");
    bNotifyTick = true;
    bNotifyTaskFinished = true;

    NextState = E_TaskState::Idle;
    PlanningTimeout = 30.0f;
    bIsRunning = false;
    ElapsedTime = 0.0f;
    CurrentAgentID = -1;
}

EBTNodeResult::Type UBTTask_RequestMAPFPath::ExecuteTask(
    UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{
    UBlackboardComponent* BB = OwnerComp.GetBlackboardComponent();
    if (!BB)
    {
        UE_LOG(LogTemp, Error, TEXT("MAPF Task: Blackboard not found"));
        return EBTNodeResult::Failed;
    }

    // Записать TaskState = "Executing" (индекс 1)
    FName TaskStateName = GetKeyName(TaskState);

    // Получить цель
    FName GoalKeyName = GetKeyName(GoalLocationKey);
    FVector Goal = BB->GetValueAsVector(GoalKeyName);

    // Получить AgentID
    FName AgentKeyName = GetKeyName(AgentIDKey);
    CurrentAgentID = BB->GetValueAsInt(AgentKeyName);

    // Проверка валидности
    if (Goal.IsNearlyZero() || CurrentAgentID < 0)
    {
        UE_LOG(LogTemp, Error, TEXT("MAPF Task: Invalid Goal=%s or AgentID=%d"),
            *Goal.ToString(), CurrentAgentID);

        if (!TaskStateName.IsNone())
        {
            //BB->SetValueAsEnum(TaskStateName, 3); // Failed
        }
        return EBTNodeResult::Failed;
    }

    // Получить Pawn
    AAIController* AIController = OwnerComp.GetAIOwner();
    if (!AIController)
    {
        UE_LOG(LogTemp, Error, TEXT("MAPF Task: No AIController"));
        return EBTNodeResult::Failed;
    }

    APawn* Pawn = AIController->GetPawn();
    if (!Pawn)
    {
        UE_LOG(LogTemp, Error, TEXT("MAPF Task: No Pawn"));
        return EBTNodeResult::Failed;
    }

    // Получить компоненты
    UPathFollowerComponent* Follower = Pawn->FindComponentByClass<UPathFollowerComponent>();
    if (!Follower)
    {
        UE_LOG(LogTemp, Error, TEXT("MAPF Task: PathFollowerComponent not found on %s"), *Pawn->GetName());
        return EBTNodeResult::Failed;
    }

    UMAPFPlanner* Planner = UMAPFPlanner::GetPlanner(Pawn);
    if (!Planner)
    {
        UE_LOG(LogTemp, Error, TEXT("MAPF Task: MAPFPlanner not found"));
        return EBTNodeResult::Failed;
    }

    // Приоритет агента (из Blackboard или по ID)
    float AgentPriority = BB->GetValueAsFloat(FName("AgentPriority"));
    if (AgentPriority <= 0.0f)
    {
        AgentPriority = static_cast<float>(CurrentAgentID) * 0.1f;
    }

    // Построить путь
    float CurrentTime = Pawn->GetWorld()->GetTimeSeconds();
    FVector StartPos = Pawn->GetActorLocation();

    UE_LOG(LogTemp, Log, TEXT("MAPF Task: Planning for Agent %d (priority %.2f) from (%.0f,%.0f) to (%.0f,%.0f)"),
        CurrentAgentID, AgentPriority, StartPos.X, StartPos.Y, Goal.X, Goal.Y);

    TArray<FVector> Path = Planner->PlanPath(
        CurrentAgentID, StartPos, Goal, CurrentTime, AgentPriority);

    if (Path.Num() < 2)
    {
        UE_LOG(LogTemp, Warning, TEXT("MAPF Task: No path found for Agent %d"), CurrentAgentID);

        if (!TaskStateName.IsNone())
        {
            //BB->SetValueAsEnum(TaskStateName, 3); // Failed
        }
        return EBTNodeResult::Failed;
    }

    // Конвертировать в Grid
    TArray<FIntVector> GridPath;
    GridPath.Reserve(Path.Num());
    for (const FVector& P : Path)
    {
        GridPath.Add(Planner->WorldToGrid(P));
    }

    // Зарезервировать путь
    if (!Planner->ReservePath(CurrentAgentID, GridPath, AgentPriority))
    {
        UE_LOG(LogTemp, Warning, TEXT("MAPF Task: Reservation failed for Agent %d, retrying with high priority"),
            CurrentAgentID);

        Path = Planner->PlanPath(CurrentAgentID, StartPos, Goal, CurrentTime, -1.0f);
        if (Path.Num() < 2)
        {
            if (!TaskStateName.IsNone())
            {
               // BB->SetValueAsEnum(TaskStateName, 3); // Failed
            }
            return EBTNodeResult::Failed;
        }

        GridPath.Empty();
        for (const FVector& P : Path)
        {
            GridPath.Add(Planner->WorldToGrid(P));
        }

        if (!Planner->ReservePath(CurrentAgentID, GridPath, -1.0f))
        {
            if (!TaskStateName.IsNone())
            {
                //BB->SetValueAsEnum(TaskStateName, 3); // Failed
            }
            return EBTNodeResult::Failed;
        }
    }

    // Установить путь
    Follower->AgentID = CurrentAgentID;
    Follower->SetPath(Path);

    UE_LOG(LogTemp, Log, TEXT("MAPF Task: Path set for Agent %d (%d points)"),
        CurrentAgentID, Path.Num());

    bIsRunning = true;
    ElapsedTime = 0.0f;

    return EBTNodeResult::InProgress;
}

void UBTTask_RequestMAPFPath::TickTask(
    UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds)
{
    if (!bIsRunning)
    {
        return;
    }

    ElapsedTime += DeltaSeconds;

    // ======== TIMEOUT ========
    if (ElapsedTime >= PlanningTimeout)
    {
        UE_LOG(LogTemp, Warning, TEXT("MAPF Task: Timeout for Agent %d (%.1f sec)"),
            CurrentAgentID, ElapsedTime);

        UBlackboardComponent* BB = OwnerComp.GetBlackboardComponent();
        if (BB)
        {
            FName TaskStateName = GetKeyName(TaskState);
            if (!TaskStateName.IsNone())
            {
                //BB->SetValueAsEnum(TaskStateName, 3); // Failed
            }
        }

        bIsRunning = false;
        FinishLatentTask(OwnerComp, EBTNodeResult::Failed);
        return;
    }

    // ======== ПРОВЕРКА КОМПОНЕНТОВ ========
    AAIController* AIController = OwnerComp.GetAIOwner();
    if (!AIController)
    {
        bIsRunning = false;
        FinishLatentTask(OwnerComp, EBTNodeResult::Failed);
        return;
    }

    APawn* Pawn = AIController->GetPawn();
    if (!Pawn)
    {
        bIsRunning = false;
        FinishLatentTask(OwnerComp, EBTNodeResult::Failed);
        return;
    }

    UPathFollowerComponent* Follower = Pawn->FindComponentByClass<UPathFollowerComponent>();
    if (!Follower)
    {
        bIsRunning = false;
        FinishLatentTask(OwnerComp, EBTNodeResult::Failed);
        return;
    }

    // ======== ПРОВЕРКА ЗАВЕРШЕНИЯ ПУТИ ========
    if (Follower->bFinished)
    {
        UBlackboardComponent* BB = OwnerComp.GetBlackboardComponent();
        if (BB)
        {
            FName TaskStateName = GetKeyName(TaskState);

            UE_LOG(LogTemp, Warning, TEXT("=== BEFORE WRITE ==="));
            UE_LOG(LogTemp, Warning, TEXT("TaskState key name: '%s'"), *TaskStateName.ToString());
            UE_LOG(LogTemp, Warning, TEXT("TaskState key is None: %d"), TaskStateName.IsNone());
            UE_LOG(LogTemp, Warning, TEXT("NextState value: %d"), static_cast<uint8>(NextState));

            // Прочитать текущее значение
            uint8 OldValue = BB->GetValueAsEnum(TaskStateName);
            UE_LOG(LogTemp, Warning, TEXT("Current TaskState value: %d"), OldValue);

            // Записать новое значение
            BB->SetValueAsEnum(TaskStateName, static_cast<uint8>(NextState));

            // Проверить что записалось
            uint8 NewValue = BB->GetValueAsEnum(TaskStateName);
            UE_LOG(LogTemp, Warning, TEXT("=== AFTER WRITE ==="));
            UE_LOG(LogTemp, Warning, TEXT("New TaskState value: %d"), NewValue);
            UE_LOG(LogTemp, Warning, TEXT("Write %s"), (NewValue == static_cast<uint8>(NextState)) ? TEXT("SUCCESS") : TEXT("FAILED"));

            // Очистить цель
            //FName GoalKeyName = GetKeyName(GoalLocationKey);
            //BB->ClearValue(GoalKeyName);
        }

        UE_LOG(LogTemp, Log, TEXT("MAPF Task: Path completed for Agent %d, TaskState=%d"),
            CurrentAgentID, static_cast<uint8>(NextState));

        bIsRunning = false;
        FinishLatentTask(OwnerComp, EBTNodeResult::Succeeded);
        return;
    }
}