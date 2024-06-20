//Implementation of dijkstra's algorithm in Java using priority queue

import java.util.*;

class Dijkstra {
    static class Edge {
        int vertex, weight;
        Edge(int v, int w) {
            vertex = v;
            weight = w;
        }
    }

    static class Graph {
        int V;
        LinkedList<Edge>[] adj;

        Graph(int V) {
            this.V = V;
            adj = new LinkedList[V];
            for (int i = 0; i < V; i++) {
                adj[i] = new LinkedList<>();
            }
        }

        void addEdge(int u, int v, int weight) {
            adj[u].add(new Edge(v, weight));
            adj[v].add(new Edge(u, weight));
        }

        void shortestPath(int src) {
            PriorityQueue<Edge> pq = new PriorityQueue<>(V, Comparator.comparingInt(edge -> edge.weight));
            int[] dist = new int[V];
            Arrays.fill(dist, Integer.MAX_VALUE);
            pq.add(new Edge(src, 0));
            dist[src] = 0;

            while (!pq.isEmpty()) {
                Edge edge = pq.poll();
                int u = edge.vertex;

                for (Edge e : adj[u]) {
                    int v = e.vertex;
                    int weight = e.weight;

                    if (dist[v] > dist[u] + weight) {
                        dist[v] = dist[u] + weight;
                        pq.add(new Edge(v, dist[v]));
                    }
                }
            }

            System.out.println("Vertex Distance from Source");
            for (int i = 0; i < V; i++) {
                System.out.println(i + " \t\t " + dist[i]);
            }
        }
    }

    public static void main(String[] args) {
        int V = 9;
        Graph g = new Graph(V);

        g.addEdge(0, 1, 4);
        g.addEdge(0, 7, 8);
        g.addEdge(1, 2, 8);
        g.addEdge(1, 7, 11);
        g.addEdge(2, 3, 7);
        g.addEdge(2, 8, 2);
        g.addEdge(2, 5, 4);
        g.addEdge(3, 4, 9);
        g.addEdge(3, 5, 14);
        g.addEdge(4, 5, 10);
        g.addEdge(5, 6, 2);
        g.addEdge(6, 7, 1);
        g.addEdge(6, 8, 6);
        g.addEdge(7, 8, 7);

        g.shortestPath(0);
    }
}
