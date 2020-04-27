Compile:
g++ -std=c++11 -O1 main.cpp network.cpp -o candidate_solution

Execute:
./candidate_solution start_charger_name end_charger_name

Output:
Print to std::out of a string in the format:
"initial charger name, first charger name, charge time in hrs, 
second charger name, charge time in hrs, …, …, goal charger name"

My Algorithm:
Based on Djikstra algorithm, I set the tentative distance of a charger as the time consumption from the start charger to this charger, including distance time consumption and charging time consumption. My rule to check if the car need to charge is, if the car want to drive from A to B, but the current battery level of the car at A is not enough, I will let the car charge the battery just enough to arrive B. That means when the car reach B, the current battery level in B is 0.
All the details will be illustrated in the main.cpp

Idea to improve:
From my experience, many Djikstra algorithm could be improved by using A* algorithm. For this challenge, if I add an estimated cost required to extend the path. But I think this way could just improve the program speed, the optimal path may not change.