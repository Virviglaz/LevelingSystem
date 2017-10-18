#include "Positions.h"
#include "data_collector.h"
#include "error_collector.h"
#include "data_logger.h"

/* Functions */
void Position_0  (PositionTypeDef * Pos);
void Position_1  (PositionTypeDef * Pos);
void Position_2  (PositionTypeDef * Pos);
void Position_3  (PositionTypeDef * Pos);
void Position_4  (PositionTypeDef * Pos);
void Position_5  (PositionTypeDef * Pos);
void Position_6  (PositionTypeDef * Pos);
void Position_7  (PositionTypeDef * Pos);
void Position_8  (PositionTypeDef * Pos);
void Position_9  (PositionTypeDef * Pos);
void Position_10 (PositionTypeDef * Pos);
void Position_11 (PositionTypeDef * Pos);
void Position_12 (PositionTypeDef * Pos);
void Position_13 (PositionTypeDef * Pos);
void Position_14 (PositionTypeDef * Pos);
void Position_15 (PositionTypeDef * Pos);
void Position_16 (PositionTypeDef * Pos);

/* Variables */
extern PositionTypeDef Position;
extern LevelingConfigStructTypeDef LevConfig;

void (*const PositionTable[])(PositionTypeDef * Pos) = {Position_0, Position_1, Position_2, 
																												Position_3, Position_4, Position_5, 
																												Position_6, Position_7, Position_8, 
																												Position_9, Position_10, Position_11, 
																												Position_12, Position_13, Position_14,
																												Position_15};

void Position_0 (PositionTypeDef * Pos)
{
	/* example of calc func */
	Pos->D1 = Pos->x;
	Pos->D2 = Pos->y;
}

void Position_1 (PositionTypeDef * Pos)
{
	/* example of calc func */
	Pos->D1 = Pos->y;
	Pos->D2 = Pos->x;
}

void Position_2 (PositionTypeDef * Pos)
{
	/* example of calc func */
	Pos->D1 = - Pos->x;
	Pos->D2 = - Pos->y;
}

void Position_3 (PositionTypeDef * Pos)
{
	/* example of calc func */
	Pos->D1 = - Pos->x;
	Pos->D2 = Pos->y;
}

void Position_4 (PositionTypeDef * Pos)
{
	/* example of calc func */
	Pos->D1 = Pos->x;
	Pos->D2 = - Pos->y;
}

void Position_5 (PositionTypeDef * Pos)
{
	/* example of calc func */
	Pos->D1 = Pos->z;
	Pos->D2 = Pos->y;
}

void Position_6 (PositionTypeDef * Pos)
{
	/* example of calc func */
	Pos->D1 = Pos->x;
	Pos->D2 = Pos->z;
}

void Position_7 (PositionTypeDef * Pos)
{
	/* example of calc func */
	Pos->D1 = Pos->z;
	Pos->D2 = Pos->x;
}

void Position_8 (PositionTypeDef * Pos)
{
	/* example of calc func */
	Pos->D1 = Pos->x;
	Pos->D2 = Pos->y;
}

void Position_9 (PositionTypeDef * Pos)
{
	/* example of calc func */
	Pos->D1 = Pos->x;
	Pos->D2 = Pos->y;
}

void Position_10 (PositionTypeDef * Pos)
{
	/* example of calc func */
	Pos->D1 = Pos->x;
	Pos->D2 = Pos->y;
}

void Position_11 (PositionTypeDef * Pos)
{
	/* example of calc func */
	Pos->D1 = Pos->x;
	Pos->D2 = Pos->y;
}

void Position_12 (PositionTypeDef * Pos)
{
	/* example of calc func */
	Pos->D1 = Pos->x;
	Pos->D2 = Pos->y;
}

void Position_13 (PositionTypeDef * Pos)
{
	/* example of calc func */
	Pos->D1 = Pos->x;
	Pos->D2 = Pos->y;
}

void Position_14 (PositionTypeDef * Pos)
{
	/* example of calc func */
	Pos->D1 = Pos->x;
	Pos->D2 = Pos->y;
}

void Position_15 (PositionTypeDef * Pos)
{
	/* example of calc func */
	Pos->D1 = Pos->x;
	Pos->D2 = Pos->y;
}

void Position_16 (PositionTypeDef * Pos)
{
	/* example of calc func */
	Pos->D1 = Pos->x;
	Pos->D2 = Pos->y;
}

void ChangeLevelingState (char State)
{
	Position.LevelingIsOn = State;
	if (State)
		LevConfig.AutoOffTimerS = AutoOffTimerInit();
}
