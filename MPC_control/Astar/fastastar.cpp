/*
File Name: main.cpp

Created by bowen wang on 2/4/20.

Copyright © 2020 bowen wang. All rights reserved.
*/

#include <math.h>
#include "global_planner/fastastar.h"
 
void Astar::InitAstar(std::vector<std::vector<int>> &_maze)
{
	maze=_maze;
}
 
int Astar::calcG(Point *temp_start,Point *point)
{
	int extraG=(abs(point->x-temp_start->x)+abs(point->y-temp_start->y))==1?kCost1:kCost2;
	return parentG+extraG;
}
 
int Astar::calcH(Point *point,Point *end)
{
	return sqrt((double)(end->x-point->x)*(double)(end->x-point->x)+(double)(end->y-point->y)*(double)(end->y-point->y))*kCost1;
}
 
int Astar::calcF(Point *point)
{
	return point->G+point->H;
}
 
Point *Astar::findPath(Point &startPoint,Point &endPoint,bool isIgnoreCorner)
{
	openList.push(new Point(startPoint.x,startPoint.y));
	while(!openList.empty())
	{
		auto curPoint=openList.top();
		openList.pop();
		curPoint->is_closed = true;
        curPoint->is_open = false;
		auto surroundPoints=getSurroundPoints(curPoint,isIgnoreCorner);
		for(auto &target:surroundPoints)
		{
			if(!target->is_open)
			{
				target->parent=curPoint;
 
				target->G=calcG(curPoint,target);
				target->H=calcH(target,&endPoint);
				target->F=calcF(target);
                target->is_open = true;
 
				openList.push(target);
                if(target->x==endPoint.x && target->y==endPoint.y) return target;
			}
			else
			{
				int tempG=calcG(curPoint,target);
				if(tempG<target->G)
				{
					target->parent=curPoint;
 
					target->G=tempG;
					target->F=calcF(target);
				}
			}
		}
	}
 
	return NULL;
}
 
std::list<Point *> Astar::GetPath(Point &startPoint,Point &endPoint,bool isIgnoreCorner)
{
	Point *result=findPath(startPoint,endPoint,isIgnoreCorner);
	std::list<Point *> path;
	while(result)
	{
		path.push_front(result);
		result=result->parent;
	}

    while (!openList.empty()) openList.pop();
	return path;
}
 
bool Astar::isCanreach(const Point *point,const Point *target,bool isIgnoreCorner) const
{
	if(target->x<0||target->x>maze.size()-1
		||target->y<0||target->y>maze[0].size()-1
		||maze[target->x][target->y]==1
		||target->x==point->x&&target->y==point->y
		||target->is_closed)
		return false;
	else
	{
		if(abs(point->x-target->x)+abs(point->y-target->y)==1)
			return true;
		else
		{
			if(maze[point->x][target->y]==0&&maze[target->x][point->y]==0)
				return true;
			else
				return isIgnoreCorner;
		}
	}
}
 
std::vector<Point *> Astar::getSurroundPoints(const Point *point,bool isIgnoreCorner) const
{
	std::vector<Point *> surroundPoints;
 
	for(int x=point->x-1;x<=point->x+1;x++)
		for(int y=point->y-1;y<=point->y+1;y++)
			if(isCanreach(point,new Point(x,y),isIgnoreCorner))
				surroundPoints.push_back(new Point(x,y));
	
	return surroundPoints;
}
