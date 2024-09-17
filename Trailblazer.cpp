/*Trailblazer.cpp
 * --------------
 *
 * This file uses multiple different search algorithms
 * in order to implement a mapping program that returns the shortest
 * path between to locations on a graph
 * Cameron Mirhossaini
 */

#include "Trailblazer.h"
#include "queue.h"
#include "priorityqueue.h"
using namespace std;




static const double SUFFICIENT_DIFFERENCE = 0.2;
Path getAltPath(Path &, PriorityQueue<Path> &);
Path searchAlgorith(const RoadGraph& , RoadNode* , RoadNode* , string);


/* General Search Algorithm
 * =================
 * Contains necessary weights for each different search algorithm
 * The only difference between BFS, Dijkstra, and A* are the weights put into
 * the priority queue. In BFS, the priority weight is only each successive weight.
 * In D, the weight is that of the edge
 * in A*, the weight is that of the edge in relation to its distance to the target. These taken into
 * conisderation, a skeleton could be created that incorporates all search algorithms
 *
 */
Path searchAlgorith(const RoadGraph& graph, RoadNode* start, RoadNode* end, string algorithm) {
    PriorityQueue<Path> queue;
    Set<RoadNode *> visited;

    int counter = 0;
    Path first;
    queue.enqueue(first += start, 0);
    double astarWeight = 0.0;

    while(!queue.isEmpty() && !visited.contains(end)) {
        double weight = queue.peekPriority();
        Path currentPath = queue.dequeue();
        RoadNode * last = currentPath[currentPath.size() - 1];
        last->setColor(Color::GREEN);
        visited.add(last);
        if(last == end) return currentPath;

        for(RoadNode * neighbor : graph.neighborsOf(last)) {
            if(!visited.contains(neighbor)) {
                neighbor->setColor(Color::YELLOW);
                Path newPath = currentPath;
                newPath += neighbor;
                double newWeight = weight + graph.edgeBetween(last, neighbor)->cost();

                if(algorithm == "BFS") queue.enqueue(newPath, counter++);
                if(algorithm == "D") queue.enqueue(newPath, newWeight);
                if(algorithm == "A*") {
                    double previousHeuristic = graph.crowFlyDistanceBetween(last, end) / graph.maxRoadSpeed();
                    astarWeight = graph.crowFlyDistanceBetween(neighbor, end) / graph.maxRoadSpeed();
                    queue.enqueue(newPath, newWeight + astarWeight - previousHeuristic);
                }
            }
        }
    }

    Path empty;
    return empty;
}




/* Breadth First Search
 * ====================
 * implements breadth-first search algorithm which looks at one hop at a time,
 * ie paths of each neighbor will be analyzed after every stack frame
 */
Path breadthFirstSearch(const RoadGraph& graph, RoadNode* start, RoadNode* end) {
    return searchAlgorith(graph, start, end, "BFS");

}

/* Dijkstra's Algorithm
 * ====================
 * implements dijkstra's algorithm which looks at individual weights
 * in order to make decisions on prospective paths
 */
Path dijkstrasAlgorithm(const RoadGraph& graph, RoadNode* start, RoadNode* end) {
 return searchAlgorith(graph, start, end, "D");

}


/* A* Algorithm
 * ====================
 * Implements A* algorithm by inserting admissable heuristic that incorporates
 * direction and distance into priority. It is faster and still finds the shortest route
 */
Path aStar(const RoadGraph& graph, RoadNode* start, RoadNode* end) {
    return searchAlgorith(graph, start, end, "A*");
}


/* Alternate Route
 * ===============
 * This function finds an alternate route by searching through the graph while excluding one edge
 * from the best solution at a time.
 */
Path alternativeRoute(const RoadGraph& graph, RoadNode* start, RoadNode* end) {
    Path bestPath = aStar(graph, start, end);
    if(bestPath.isEmpty()) return bestPath;

    PriorityQueue<Path> altQueue;
//goes through every edge
    for(int i = 1; i < bestPath.size(); i++) {
        PriorityQueue<Path> queue;
        Set<RoadNode *> visited;

        Path first;
        queue.enqueue(first += start, 0);
        double astarWeight = 0.0;

        while(!queue.isEmpty() && !visited.contains(end)) {
            double weight = queue.peekPriority();
            Path currentPath = queue.dequeue();
            RoadNode * last = currentPath[currentPath.size() - 1];
            last->setColor(Color::GREEN);
            visited.add(last);
            //instead of returning, will keep adding paths
            if(last == end) altQueue.enqueue(currentPath, weight);
            for(RoadNode * neighbor : graph.neighborsOf(last)) {
                if(!visited.contains(neighbor)) {
                    //checks each edge that isn't contained
                    if(neighbor != bestPath[i] || last != bestPath[i - 1]) {
                        neighbor->setColor(Color::YELLOW);
                        Path newPath = currentPath;
                        double previousHeuristic = graph.crowFlyDistanceBetween(last, end) / graph.maxRoadSpeed();
                        double newWeight = weight + graph.edgeBetween(last, neighbor)->cost();
                        astarWeight = graph.crowFlyDistanceBetween(neighbor, end) / graph.maxRoadSpeed();
                        queue.enqueue(newPath += neighbor, newWeight + astarWeight - previousHeuristic);
                    }
                }
            }
        }
    }
    return getAltPath(bestPath, altQueue);
}




/* Get Alt Path
 * ============
 * This function uses all alternate paths it has collected from the previous
 * search algorithm and finds the amount of similarity between them and the best path.
 * The shortest with the least amount of similarity will be returned as the alternate path
 */
Path getAltPath(Path & bestPath, PriorityQueue<Path> & altQueue) {

    Set<RoadNode *> bestNodeSet;
    //makes set of all nodes in original best path
    for(int i = 0; i < bestPath.size(); i++) {
        bestNodeSet += bestPath[i];
    }
//makes set for each next alternative best path
//gets difference of two sets and returns path if difference is less than .2
    while(!altQueue.isEmpty()) {
        Path next = altQueue.dequeue();
        Set<RoadNode *> altNodeSet;

        for(int i = 0; i < next.size(); i++) {
            altNodeSet.add(next[i]);
        }

        Set<RoadNode *> difference = altNodeSet - bestNodeSet;
        if(((double)difference.size() / (double)bestPath.size()) > SUFFICIENT_DIFFERENCE) return next;
    }

    Path empty;
    return empty;
}

