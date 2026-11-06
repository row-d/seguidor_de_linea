#ifndef ROUTE_MEMORY_H
#define ROUTE_MEMORY_H

#include <stddef.h>
#include <stdint.h>

enum Route
{
    ROUTE_LEFT,
    ROUTE_RIGHT
};

// Que hacer al volver a una bifurcacion despues de un callejon sin salida.
struct ReturnDecision
{
    enum Action
    {
        TAKE_OTHER_BRANCH,
        BACK_TO_STEM,
        NO_ROUTE_LEFT
    };

    Action action;
    // De vuelta en la Y, la rama pendiente y la entrada se alcanzan girando hacia
    // el lado de la primera rama: sin pasar por la rama de la que viene el robot.
    Route turn;
    Route other_branch;
    unsigned long stem_ms;
};

// Pila de bifurcaciones abiertas. Marca que ramas de cada una ya se visitaron.
template <size_t Capacity>
class RouteMemory
{
private:
    struct ForkVisit
    {
        Route first_route;
        bool both_visited;
        unsigned long stem_ms;
    };

    ForkVisit forks[Capacity];
    uint8_t count = 0;

public:
    size_t depth() const
    {
        return count;
    }

    // stem_ms: tiempo que tardo en llegar desde la bifurcacion anterior.
    bool push(Route first_route, unsigned long stem_ms)
    {
        if (count >= Capacity)
            return false;

        forks[count++] = {first_route, false, stem_ms};
        return true;
    }

    ReturnDecision on_return()
    {
        ForkVisit &fork = forks[count - 1];
        const Route other = fork.first_route == ROUTE_LEFT ? ROUTE_RIGHT : ROUTE_LEFT;

        if (!fork.both_visited)
        {
            fork.both_visited = true;
            return {ReturnDecision::TAKE_OTHER_BRANCH, fork.first_route, other, 0};
        }

        const ReturnDecision decision = {
            count > 1 ? ReturnDecision::BACK_TO_STEM : ReturnDecision::NO_ROUTE_LEFT,
            fork.first_route, other, fork.stem_ms};
        --count;
        return decision;
    }
};

#endif
