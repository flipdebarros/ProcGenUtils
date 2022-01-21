using System;
using System.Collections.Generic;
using System.Linq;
using JetBrains.Annotations;
using Utils.ProcGenUtils.DataStructures;

namespace Utils.ProcGenUtils.GraphModel.SpanningTrees {

public static class SpanningTreeAlgorithms {

    /// <summary>
    /// Finds the minimal spanning tree of a weighted undirected graph using the Prim algorithm. 
    /// </summary>
    /// <param name="graph">Graph object</param>
    /// <param name="weightSelector">Function that selects the weight of given edge</param>
    /// <param name="maxWeight">TWeight equivalent of positive infinity</param>
    /// <param name="minWeight">TWeight equivalent of negative infinity</param>
    /// <param name="seed">Starting vertex key</param>
    /// <typeparam name="TKey">Type that will be used to index the vertices.</typeparam>
    /// <typeparam name="TValue">Type that vertices hold as value</typeparam>
    /// <typeparam name="TWeight">Comparable type of the edge weights</typeparam>
    /// <returns>A graph object containing a minimal spanning tree of the graph</returns>
    /// <exception cref="InvalidOperationException">Graph cannot be directed</exception>
    public static Graph<TKey, TValue> Prim<TKey, TValue, TWeight>( this Graph<TKey, TValue> graph, 
        [NotNull] Func<TKey, TKey, TWeight> weightSelector, 
        [NotNull] TWeight maxWeight, [NotNull] TWeight minWeight, TKey seed) where TWeight : IComparable {
        if (weightSelector == null) throw new ArgumentNullException(nameof(weightSelector));
        if (graph.VertexCount == 0)
            return new Graph<TKey, TValue>(graph.DefaultWeight);
        
        if (graph.IsDirected)
            throw new InvalidOperationException("Graph cannot be directed.");

        if (!graph.ContainsVertex(seed))
            throw new ArgumentException("Given seed not in graph.");
        
        var tree = new Graph<TKey, TValue>(graph.DefaultWeight);
        var heap = new FibonacciHeap<TKey, TWeight>();
        var pi = new Dictionary<TKey, TKey>();
        graph.Vertices.ForEach(v => {
            tree[v] = graph[v];
            heap[v] = maxWeight;
            pi[v] = v;
        });

        heap[seed] = minWeight;
        while (heap.Count > 0) {
            var u = heap.ExtractMin();
            foreach (var v in graph.GetAdjacency(u)) {
                if (!heap.Contains(v) || weightSelector(u, v).CompareTo(heap[v]) >0 ) continue;
                pi[v] = u;
                heap[v] = weightSelector(u, v);
            }

            if (!pi[u].Equals(u)) 
                tree[pi[u], u, EdgeType.Tree] = graph[pi[u], u];
        }
        
        return tree;
    }

    /// <summary>
    /// Finds the minimal spanning tree of a weighted undirected graph using the Prim algorithm. 
    /// </summary>
    /// <param name="graph">Graph object</param>
    /// <param name="weightSelector">Function that selects the weight of given edge</param>
    /// <param name="maxWeight">TWeight equivalent of positive infinity</param>
    /// <param name="minWeight">TWeight equivalent of negative infinity</param>
    /// <typeparam name="TKey">Type that will be used to index the vertices.</typeparam>
    /// <typeparam name="TValue">Type that vertices hold as value</typeparam>
    /// <typeparam name="TWeight">Comparable type of the edge weights</typeparam>
    /// <returns>A graph object containing a minimal spanning tree of the graph</returns>
    /// <exception cref="InvalidOperationException">Graph cannot be directed</exception>
    public static Graph<TKey, TValue> Prim<TKey, TValue, TWeight>(this Graph<TKey, TValue> graph,
        [NotNull] Func<TKey, TKey, TWeight> weightSelector,
        [NotNull] TWeight maxWeight, [NotNull] TWeight minWeight) where TWeight : IComparable =>
        graph.Prim(weightSelector, maxWeight, minWeight, graph.Vertices[0]);
    
    /// <summary>
    /// Finds the minimal spanning tree of a weighted undirected graph using the Prim algorithm. 
    /// </summary>
    /// <param name="graph">Graph object</param>
    /// <typeparam name="TKey">Type that will be used to index the vertices.</typeparam>
    /// <typeparam name="TValue">Type that vertices hold as value</typeparam>
    /// <returns>A graph object containing a minimal spanning tree of the graph</returns>
    /// <exception cref="InvalidOperationException">Graph cannot be directed</exception>
    public static Graph<TKey, TValue> Prim<TKey, TValue>(this Graph<TKey, TValue> graph) =>
        Prim(graph, (u, v) => graph[u, v], float.PositiveInfinity, float.NegativeInfinity, graph.Vertices[0]);
    
    /// <summary>
    /// Finds the maximal spanning tree of a weighted undirected graph using the Prim algorithm. 
    /// </summary>
    /// <param name="graph">Graph object</param>
    /// <typeparam name="TKey">Type that will be used to index the vertices.</typeparam>
    /// <typeparam name="TValue">Type that vertices hold as value</typeparam>
    /// <returns>A graph object containing a maximal spanning tree of the graph</returns>
    /// <exception cref="InvalidOperationException">Graph cannot be directed</exception>
    public static Graph<TKey, TValue> PrimMaxTree<TKey, TValue>(this Graph<TKey, TValue> graph) =>
        Prim(graph, (u, v) => -graph[u, v], float.PositiveInfinity, float.NegativeInfinity, graph.Vertices[0]);

    /// <summary>
    /// Finds the minimal spanning tree of a weighted undirected graph using the Kruskal algorithm. 
    /// </summary>
    /// <param name="graph">Graph object</param>
    /// <param name="weightSelector">Function that selects the weight of given edge</param>
    /// <typeparam name="TKey">Type that will be used to index the vertices.</typeparam>
    /// <typeparam name="TValue">Type that vertices hold as value</typeparam>
    /// <typeparam name="TWeight">Comparable type of the edge weights</typeparam>
    /// <returns>A graph object containing a minimal spanning tree of the graph</returns>
    /// <exception cref="InvalidOperationException">Graph cannot be directed</exception>
    public static Graph<TKey, TValue> Kruskal<TKey, TValue, TWeight> (this Graph<TKey, TValue> graph,
        [NotNull] Func<(TKey, TKey), TWeight> weightSelector) where TWeight : IComparable {
        if (weightSelector == null) throw new ArgumentNullException(nameof(weightSelector));
        if (graph.IsDirected)
            throw new InvalidOperationException("Graph cannot be directed.");

        var tree = new Graph<TKey, TValue>(graph.DefaultWeight);
        graph.Vertices.ForEach(v => tree[v] = graph[v]);

        var set = new DisjointSets<TKey>(graph.Vertices);
        var edges = graph.Edges.OrderBy(weightSelector).ToList();
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
    /// <typeparam name="TValue">Type that vertices hold as value</typeparam>
    /// <returns>A graph object containing a minimal spanning tree of the graph</returns>
    /// <exception cref="InvalidOperationException">Graph cannot be directed</exception>
    public static Graph<TKey, TValue> Kruskal<TKey, TValue> (this Graph<TKey, TValue> graph) => 
        Kruskal(graph, edge => graph[edge].Weight);
    
    /// <summary>
    /// Finds the maximal spanning tree of a weighted undirected graph using the Kruskal algorithm. 
    /// </summary>
    /// <param name="graph">Graph object</param>
    /// <typeparam name="TKey">Type that will be used to index the vertices.</typeparam>
    /// <typeparam name="TValue">Type that vertices hold as value</typeparam>
    /// <returns>A graph object containing a maximal spanning tree of the graph</returns>
    /// <exception cref="InvalidOperationException">Graph cannot be directed</exception>
    public static Graph<TKey, TValue> KruskalMaxTree<TKey, TValue> (this Graph<TKey, TValue> graph) => 
        Kruskal(graph, edge => -graph[edge].Weight);
}
}
