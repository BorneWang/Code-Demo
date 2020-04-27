/*
 File Name: main.cpp
 
 Created by bowen wang on 2/27/20.
 
 Copyright © 2020 bowen wang. All rights reserved.
 */

#include "network.h"
#include <map>
#include <math.h>
#include <vector>
#include <unordered_map>
using namespace std;

#define _USE_MATH_DEFINES // To use const math.PI

/*////////////////////////////////////////////////////////////////
                            Assumptions
 ///////////////////////////////////////////////////////////////*/
const double Earth_radius = 6356.752;  // km
const double Speed = 105.0;            // km/hr
const double Full_Charge = 320.0;      // km

/*////////////////////////////////////////////////////////////////
                           Minheap Template
 Usage: min heap is used as a priority queue to get the minimum
 "distance" in Djikstra algorithm
 
 Citation: cite this template from:
 https://www.cnblogs.com/LowBee/p/9024928.html
 And make some small changes for my algorithm
 
 Important members and functions:
 
 1) T& RemoveTop()
 Get the node with the minimum "distance" and remove it from heap
 
 2) T& getcase(int position);
 Return the node at the specific position from heapArray.
 
 3) heapArray
 The array to store nodes
 
 4) friend class Djikstra
 Set class Djikstra as friend to make it have access to private
 members.
///////////////////////////////////////////////////////////////*/
template <class T>
class MinHeap{
private:
    friend class Djikstra;
    T *heapArray;
    int CurrentSize;
    int MaxSize;
    void swap(int pos_x,int pos_y)
    {
        T temp;
        temp=heapArray[pos_x];
        heapArray[pos_x]=heapArray[pos_y];
        heapArray[pos_y]=temp;
    }
    void BuildHeap();
public:
    MinHeap();
    bool isEmpty();
    bool isLeaf(int pos) const;
    int left(int pos) const;
    int right(int pos) const;
    int Parent(int pos) const;
    bool Remove(int pos,T &node);
    bool Remove(int pos);
    bool Insert(const T value);
    T& RemoveTop();     //Get the node with the minimum "distance" and remove it from heap
    void ShiftUp(int position);
    void ShiftDown(int left);
    T& getcase(int position);
    void display()
    {
        cout << "MaxSize:" <<MaxSize<<endl;
        cout << "CurrentSize:" <<CurrentSize<<endl;
        for(int i=0;i<CurrentSize;i++)
        {
            cout << heapArray[i] << " ";
        }
        cout <<endl;
    }

};
template <class T>
T& MinHeap<T>::getcase(int position)
{
    return heapArray[position];
}
template <class T>
MinHeap<T>::MinHeap()
{
    //if(n<0)return;
    MaxSize=303;
    CurrentSize=0;
    heapArray=new T[MaxSize];
    BuildHeap();
}
template <class T>
bool MinHeap<T>::isLeaf(int pos) const{
    return (pos>=CurrentSize/2)&&(pos<CurrentSize);
}
template <class T>
void MinHeap<T>::BuildHeap(){
    for(int i=CurrentSize/2-1;i>=0;i--)
    {
        ShiftDown(i);
    }
}
template <class T>
int MinHeap<T>::left(int pos) const
{
    return 2*pos+1;
}
template <class T>
int MinHeap<T>::right(int pos) const
{
    return 2*pos+2;
}
template <class T>
int MinHeap<T>::Parent(int pos) const
{
    return ((pos-1)/2);
}
template <class T>
bool MinHeap<T>::Insert(const T value)
{
    if(CurrentSize==MaxSize)
        return false;
    heapArray[CurrentSize]=value;
    ShiftUp(CurrentSize);
    CurrentSize++;
    return true;
}
template <class T>
T& MinHeap<T>::RemoveTop()
{
    if(CurrentSize==0)
    {
        cout << "Heap is empty" <<endl;
        return heapArray[CurrentSize-1];
    }
    else
    {
        swap(0,--CurrentSize);
        if(CurrentSize>1)
            ShiftDown(0);
        return heapArray[CurrentSize];
    }
}
template <class T>
bool MinHeap<T>::Remove(int pos,T &node)
{
    if((pos<0)||(pos>=CurrentSize))return false;
    node=heapArray[pos];
    heapArray[pos]=heapArray[--CurrentSize];
    if(heapArray[Parent(pos)]>heapArray[pos])
        ShiftUp(pos);
    else ShiftDown(pos);
    return true;
}
template <class T>
void MinHeap<T>::ShiftUp(int pos)
{
    int temppos=pos;
    T temp=heapArray[temppos];
    while((temppos>0)&&(heapArray[Parent(temppos)]>temp))
    {
        heapArray[temppos]=heapArray[Parent(temppos)];
        temppos=Parent(temppos);
    }
    heapArray[temppos]=temp;
}
template <class T>
void MinHeap<T>::ShiftDown(int l)
{
    int i=l;
    int j=left(i);
    T temp=heapArray[i];
    while(j<CurrentSize)
    {
        if((j<CurrentSize-1)&&(heapArray[j]>heapArray[j+1]))
        {
            j++;
        }
        if(temp>heapArray[j])
        {
            heapArray[i]=heapArray[j];
            i=j;
            j=left(j);
        }
        else break;
    }
    heapArray[i]=temp;
}

/*////////////////////////////////////////////////////////////////
                      Harversine Distance
 Usage: use Harversine formula to calculate the distance between
 two points depend on their latitude and longitude.
 The input are (lat1, lon1) and (lat2, lon2) in degree.
 And the function Distance will return the distance in km.
 
 Reference:
 https://en.wikipedia.org/wiki/Haversine_formula
///////////////////////////////////////////////////////////////*/
double ConvertDegreesToRadians(double degrees)
{
    return degrees * M_PI / 180;
}
double HaverSin(double theta)
{
    double v = sin(theta/2);
    return v*v;
}
double Distance(double lat1, double lon1, double lat2, double lon2)
{
    lat1 = ConvertDegreesToRadians(lat1);
    lon1 = ConvertDegreesToRadians(lon1);
    lat2 = ConvertDegreesToRadians(lat2);
    lon2 = ConvertDegreesToRadians(lon2);
    
    double vLon = abs(lon1 - lon2);
    double vLat = abs(lat1 - lat2);
    
    double h = HaverSin(vLat) + cos(lat1) * cos(lat2) * HaverSin(vLon);
    
    
    double temp = asin(sqrt(h));
    double distance = temp * 2 * Earth_radius;
    
    return distance;
}

/*////////////////////////////////////////////////////////////////
                           Struct Edge
 Usage: use Edge to store the relation between two chargers from
 network
 
 Members:
 string start : start charger of the edge.
 string end : end charger of the edge.
 double time_consume : time consumption from start to end. Since
                        Speed of the car is const. This value woule
                        be Distance/Speed. And this value could represent
                        the "distance" between two chargers.
 double start_rate : the charge rate of the start charger.
 double end_rate : the charge rate of the end charger.
 
 Note:
 1) Since the speed of the car is const, the start_rate and end_rate
 are Speed/real_rate instead of real charge rate itself.
 For example: the real charger rate of charger Albany_NY is 131.0 km/hr
 and the Speed of the car is 105 km/hr. So the start_rate here is
 105/131 = 0.8015.
 
 2) For two chargers A and B. The program will create two Edge.
 One is from A to B, the other is from B to A.
///////////////////////////////////////////////////////////////*/
struct Edge
{
    string start;
    string end;
    double time_consume;
    double start_rate; // usage: (time_consum - current_time_left) * start_rate
    double end_rate;
};

/*////////////////////////////////////////////////////////////////
                           Class Graph
 Usage: use Graph to pre-process the network. Extract and store the Edge
 information from network.
 
 Pre-process:
 For each charger of the network, travel the rest chargers and calculate
 the Distance. If the Distance is less than Full_Charge, create the Edge and
 insert to the graph.
 For example: given charger A in the network, travel the rest chargers, for
 example charger B. Calculate the distance between A and B. If the distance is
 less than Full_Charge, that means the car starting from A with full battery
 could reach B, also, starting from B with full battery could reach A. Therefore,
 create Edge(A,B) and Edge(B,A). And store them to the graph.
///////////////////////////////////////////////////////////////*/
class Graph
{
public:
    Graph();
    Graph(array<row, 303> network);
    map<string, vector<Edge>> graph;
    int size();
};
Graph::Graph(array<row, 303> network)
{
    for(int i=0; i<302; ++i)
    {
        for(int j=i+1; j<303; ++j)
        {
            double distance = Distance(network[i].lat, network[i].lon, network[j].lat, network[j].lon);
            if(distance <= Full_Charge)
            {
                Edge e = {network[i].name, network[j].name, distance/Speed, Speed/network[i].rate};
                graph[network[i].name].push_back(e);
                e = {network[j].name, network[i].name, distance/Speed, Speed/network[j].rate};
                graph[network[j].name].push_back(e);
            }
        }
    }
}

/*////////////////////////////////////////////////////////////////
                        struct DjikstraNode
 Usage: the node used in Djikstra Algorithm and also uesd for min
 heap sort.
 
 Members:
 string name : charger name;
 double Current_Battery : if the car drives from start_charger to this
                         charger, the Current_Battery represent how much
                         energy left in the battery at this charger. Since
                        the Speed of the car is const, this value here is a
                        time value with unit hr.
 string parent : the last charger the car visited before arriving this charger.
 double time_consume : how much time used from start_charger to this charger,
                        including distance time consumption and charging time
                        consumption.
 double start_rate : the charge rate of this charger if the car need to go from
                    this charger to another charger.
 double parent_charge_time : before arriving this charger, how much time was used
                            at the parent charger to charge the car.
 bool operator > : used for min heap sort.
///////////////////////////////////////////////////////////////*/
struct DjikstraNode
{
    string name;
    double Current_Battery;
    string parent;
    double time_consume;
    double start_rate; // usage: (time_consum - current_time_left) * start_rate
    double parent_charge_time = 0;
    bool operator >  (const DjikstraNode temp) const
    {
        return time_consume > temp.time_consume;
    }
};
/*////////////////////////////////////////////////////////////////
                           class Djikstra
 Usage : use this class to run Djikstra Algorithm and print results.
 
 Description:
 1) The input is the name of start_charger and end_charger, and the network
 2) The initialization function Djikstra will build graph from network,
    then, set the start_charger as visited. Then, extract all the chargers
    that could be arrived from the start_charger from the graph, and put them to
    the min heap.
 3) The function run will keep doing Djikstra Algorithm until the end_charger
    is visited.
    The procedure is : the min heap will give out the next charger with the minimum
    time consumption to reach there. Then, set that charger as visited. Then, add new
    Edge information depending on that charger to the min heap. Then, update all the
    time consumption value, parent value and current_battery value depending on
    the new Edge information. This step is as same as the General Djikstra Algorithm
 4) Once the end_charger is visited. The function display will print an optimal
    path with charging time.
 
 Members:
 string start_name : start charger
 string goal_name : end charger
 map<string, vector<Edge>> graph : store all the Edge information from network
 MinHeap<DjikstraNode> minheap : min heap used for giving out minimum charger
                                 also store all the unvisited but reachable charger
 unordered_map<string, string> visited : store all the visited charger
                                        format -> (charger, parent charger)
 unordered_map<string, double> charge_time : store the parent charing time to
                                             arrive to this charger.
                format -> (charger A, A's parent charger's charging time)
 
 All the functions' illustration can be found before the function's definition.
 
///////////////////////////////////////////////////////////////*/
class Djikstra
{
public:
    string start_name;
    string goal_name;
    map<string, vector<Edge>> graph;
    MinHeap<DjikstraNode> minheap;
    unordered_map<string, string> visited;
    unordered_map<string, double> charge_time;
    Djikstra();
    Djikstra(string initial_charger_name, string goal_charger_name, array<row, 303> network);
    void run();
    void update(DjikstraNode step);
    bool ifvisited(string name);
    int find_in_minheap(string name);
    void display();
};
/*////////////////////////////////////////////////////////////////
                    void Djikstra::display()
 print an optimal path from start charger to the end charger
///////////////////////////////////////////////////////////////*/
void Djikstra::display()
{
    cout<<"\""<<start_name<<", ";
    string cur = goal_name;
    vector<string> results;
    while(1)
    {
        if(cur == start_name) break;
        results.push_back(visited[cur]);
        cur = visited[cur];
    }
    int length = results.size();
    if(length == 1)
    {
        cout<<"start and end are same"<<'\n';
        return;
    }
    cout<<results[length-2]<<", ";
    for(int i=length-3; i>=0; --i)
    {
        cout<<charge_time[results[i]]<<", "<<results[i]<<", ";
    }
    cout<<charge_time[goal_name]<<", "<<goal_name<<"\""<<'\n';
}
/*////////////////////////////////////////////////////////////////
            int Djikstra::find_in_minheap(string name)
 Check if the charger is in the min heap, if founded, reture its
 index, if not, reture -1.
///////////////////////////////////////////////////////////////*/
int Djikstra::find_in_minheap(string name)
{
    for(int i=0; i<minheap.CurrentSize; ++i)
    {
        if(name == minheap.heapArray[i].name) return i;
    }
    return -1;
}
/*////////////////////////////////////////////////////////////////
                bool Djikstra::ifvisited(string name)
 Check if the charger is visited, return true or false
///////////////////////////////////////////////////////////////*/
bool Djikstra::ifvisited(string name)
{
    unordered_map<string, string>::iterator it;
    it = visited.find(name);
    if(it != visited.end()) return true;
    else return false;
}
/*////////////////////////////////////////////////////////////////
                void Djikstra::update(DjikstraNode step)
 The input is a DjikstraNode, this should be the output of min heap.
 That means this charger has the minimum time consumption.
 
 Then, from graph extract all the Edges starting from this charger i.e. A.
 For each Edge, check if the end charger i.e B is visited, if true, continue.
 
 Then, check if the end charger is in the min heap, if true, update that
 node depending on the Edge information. The rule is, calculate the charging
 time in A if the car want to go B but the current battery is not enough. Then,
 calculate the time consumption from start charger to B with passing A. If this
 new time consumption is less than the old one in the min heap, update that node
 with new time consumption, new parent, new current battery and new parent charge
 time.
 
 If the end charger B is not in the min heap, insert it to the min heap.
///////////////////////////////////////////////////////////////*/
void Djikstra::update(DjikstraNode step)
{
    vector<Edge> edges = graph[step.name];
    for(int i=0; i<edges.size(); ++i)
    {
        if(ifvisited(edges[i].end)) continue;
        int position = find_in_minheap(edges[i].end);
        
        double current_battery = 0;
        double charging_time = 0;
        if(step.Current_Battery >= edges[i].time_consume)
        {
            current_battery = step.Current_Battery - edges[i].time_consume;
        }
        else
        {
            charging_time = ( edges[i].time_consume - step.Current_Battery ) * step.start_rate;
        }
        double time_consume = step.time_consume + edges[i].time_consume + charging_time;
        
        if(position == -1) // this is a new charger
        {
            DjikstraNode node;
            node.name = edges[i].end;
            node.Current_Battery = current_battery;
            node.parent = step.name;
            node.time_consume = time_consume;
            node.start_rate = edges[i].end_rate;
            node.parent_charge_time = charging_time;
            minheap.Insert(node);
        }
        else  // this charger already in the min heap and need to update
        {
            DjikstraNode node = minheap.getcase(position);
            if(time_consume < node.time_consume)
            {
                node.time_consume = time_consume;
                node.parent = step.name;
                node.Current_Battery = current_battery;
                node.parent_charge_time = charging_time;
            }
        }
    }
    visited[step.name] = step.parent;
    charge_time[step.name] = step.parent_charge_time;
};
/*////////////////////////////////////////////////////////////////
                   void Djikstra::run()
 Keep giving out the minimum node from min heap and update it.
 Unitl the end charger has been found.
 Then, display the results.
///////////////////////////////////////////////////////////////*/
void Djikstra::run()
{
    while(1)
    {
        DjikstraNode step = minheap.RemoveTop();
        if(step.name == goal_name){
            visited[step.name] = step.parent;
            charge_time[step.name] = step.parent_charge_time;
            break;
        }
        update(step);
    }
    string cur = goal_name;
    display();
}
/*////////////////////////////////////////////////////////////////
                   Djikstra::Djikstra
 Build graph from network, then, set the start_charger as visited. Then, extract all
 the chargers that could be arrived from the start_charger from the graph, and put them to
 the min heap.
///////////////////////////////////////////////////////////////*/
Djikstra::Djikstra(string initial_charger_name, string goal_charger_name, array<row, 303> network)
{
    start_name = initial_charger_name;
    goal_name = goal_charger_name;
    
    //build graph from network
    for(int i=0; i<302; ++i)
    {
        for(int j=i+1; j<303; ++j)
        {
            double distance = Distance(network[i].lat, network[i].lon, network[j].lat, network[j].lon);
            if(distance <= Full_Charge)
            {
                Edge e = {network[i].name, network[j].name, distance/Speed, Speed/network[i].rate, Speed/network[j].rate};
                graph[network[i].name].push_back(e);
                e = {network[j].name, network[i].name, distance/Speed, Speed/network[j].rate, Speed/network[i].rate};
                graph[network[j].name].push_back(e);
            }
        }
    }
    
    //
    unsigned long n = graph[start_name].size();
    vector<Edge> edges = graph[start_name];
    for(int i=0; i<n; ++i)
    {
        DjikstraNode node;
        node.name = edges[i].end;
        node.Current_Battery = Full_Charge/Speed-edges[i].time_consume;
        node.parent = start_name;
        node.time_consume = edges[i].time_consume;
        node.start_rate = edges[i].end_rate;
        minheap.Insert(node);
    }
    visited[start_name] = "None";
}




/*////////////////////////////////////////////////////////////////
                           Main
///////////////////////////////////////////////////////////////*/
int main(int argc, char** argv)
{
    
    
    if (argc != 3)
    {
        std::cout << "Error: requires initial and final supercharger names" << std::endl;        
        return -1;
    }
    
    string initial_charger_name = argv[1];
    string goal_charger_name = argv[2];
    
    Djikstra D1(initial_charger_name, goal_charger_name, network);
    D1.run();

    return 0;
}
