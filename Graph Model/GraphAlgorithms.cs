using System;
using System.Collections.Generic;
using System.Linq;
using Object = UnityEngine.Object;

namespace Utils.ProcGenUtils.GraphModel {

public static class GraphAlgorithms {
    
    /// <summary>
    /// Finds a topological order for a directed acyclic graph.
    /// </summary>
    /// <param name="graph">Graph object</param>
    /// <typeparam name="TKey">Type that will be used to index the vertices.</typeparam>
    /// <typeparam name="TValue">Type that vertices will hold as value</typeparam>
    /// <returns>List of vertices keys in the topological order.</returns>
    /// <exception cref="InvalidOperationException">Graph cannot be undirected, topological sort only works for directed acyclic graphs.</exception>
    /// <exception cref="InvalidOperationException">"Back edge found. Graph cannot be cyclical, topological sort only works for directed acyclic graphs"</exception>
    public static List<TKey> TopologicalSort<TKey, TValue>(this Graph<TKey, TValue> graph) {
        if (!graph.IsDirected)
            throw new InvalidOperationException("Graph cannot be undirected, topological sort only works for directed acyclic graphs.");
        var res = new LinkedList<TKey>();
        
        graph.DepthFirstSearch(graph.Vertices, u => {}, (u, v) => {}, 
            (u, v) => throw new InvalidOperationException("Back edge found: " + (u, v) +". Graph cannot be cyclical, topological sort only works for directed acyclic graphs"),
            (u, v) => {}, u => res.AddFirst(u));
        
        return res.ToList();
    }

    /// <summary>
    /// Finds all the strongly connected components of a directed graph.
    /// </summary>
    /// <param name="graph">Graph object</param>
    /// <typeparam name="TKey">Type that will be used to index the vertices.</typeparam>
    /// <typeparam name="TValue">Type that vertices will hold as value</typeparam>
    /// <returns>A list of all strongly connected components</returns>
    /// <exception cref="InvalidOperationException"></exception>
    public static List<List<TKey>> StronglyConnectedComponents<TKey, TValue>(this Graph<TKey, TValue> graph) {
        if (!graph.IsDirected)
            throw new InvalidOperationException("Graph cannot be undirected.");

        var time = 0;
        var finishTime = graph.Vertices.ToDictionary(v => v, v => -1); 
        
        //Calculate Finish Times
        graph.DepthFirstSearch(u => time++, u => finishTime[u] = ++time);

        var vertices = graph.Vertices;
        vertices.Sort((u, v) => finishTime[v] - finishTime[u]);

        var explored = vertices.ToDictionary(v => v, v => false);
        var scc = new List<List<TKey>>();
        var component = new List<TKey>();

        void Discover(TKey u) {
            component.Add(u);
        }

        void Finish(TKey u) {
            if (explored[u]) return; 
            
            /* if u was not explored, then its the root of a strongly connected component
            and that means every vertex of this component has been added, so add the component
            to the list of components and start a new one. */
            scc.Add(component);
            component = new List<TKey>();
        }
        
        graph.Transpose.DepthFirstSearch(vertices, Discover, (u, v) => explored[v] = true, Finish);
        
        return scc;
    }

    private static Graph<TKey, TValue> Kruskal<TKey, TValue> (Graph<TKey, TValue> graph, Comparison<(TKey, TKey)> comparison) {
        if (graph.IsDirected)
            throw new InvalidOperationException("Graph cannot be directed.");

        var tree = new Graph<TKey, TValue>(graph.DefaultWeight);
        graph.Vertices.ForEach(v => tree[v] = graph[v]);

        var set = new DisjointSets<TKey>(graph.Vertices);
        var edges = graph.Edges;
        edges.Sort(comparison);
        edges.ForEach(tuple => {
            var (u, v) = tuple;
            if (set[u].Equals(set[v])) return;
            tree[u, v, EdgeType.Tree] = graph[u, v];
            set[u] = v;
        });
        return tree;
    }

    /// <summary>
    /// Finds the minimal spanning tree of a weighted undirected graph using the Kruskal algorithm. 
    /// </summary>
    /// <param name="graph">Graph object</param>
    /// <typeparam name="TKey">Type that will be used to index the vertices.</typeparam>
    /// <typeparam name="TValue">Type that vertices will hold as value</typeparam>
    /// <returns>A graph object containing a minimal spanning tree of the graph</returns>
    /// <exception cref="InvalidOperationException">Graph cannot be directed</exception>
    public static Graph<TKey, TValue> MinSpanningTreeKruskal<TKey, TValue> (this Graph<TKey, TValue> graph) => 
        Kruskal(graph, (e1, e2) => Math.Sign(graph[e1].Weight - graph[e2].Weight));
    
    /// <summary>
    /// Finds the maximal spanning tree of a weighted undirected graph using the Kruskal algorithm. 
    /// </summary>
    /// <param name="graph">Graph object</param>
    /// <typeparam name="TKey">Type that will be used to index the vertices.</typeparam>
    /// <typeparam name="TValue">Type that vertices will hold as value</typeparam>
    /// <returns>A graph object containing a maximal spanning tree of the graph</returns>
    /// <exception cref="InvalidOperationException">Graph cannot be directed</exception>
    public static Graph<TKey, TValue> MaxSpanningTreeKruskal<TKey, TValue> (this Graph<TKey, TValue> graph) => 
        Kruskal(graph, (e1, e2) => Math.Sign(graph[e2].Weight - graph[e1].Weight));
}

}
