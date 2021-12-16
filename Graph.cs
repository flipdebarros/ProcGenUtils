using System;
using System.Collections.Generic;
using System.Linq;
using JetBrains.Annotations;
using Unity.VisualScripting;
using UnityEngine;

namespace Utils.ProcGenUtils.GraphModel {
/// <summary>
/// A container that implements the mathematical model of a generic weighted graph,
/// it can be directed or undirected
/// </summary>
/// <typeparam name="TKey">Type that will be used to index the vertices.</typeparam>
/// <typeparam name="TValue">Type that vertices will hold as value</typeparam>
public class Graph<TKey, TValue> {
    private Dictionary<TKey, TValue> _vertices;
    private Dictionary<TKey, List<TKey>> _adjacency;
    private Dictionary<TKey, List<TKey>> _transpose;
    private Dictionary<KeyPair<TKey>, Edge<TKey>>_edges;

    #region Public Properties

    /// <summary>
    /// Returns o list of all pairs of keys that define an edge in the graph.
    /// </summary>
    public List<(TKey, TKey)> Edges => _edges.Select(pair => pair.Key.Tuple).ToList();
    /// <summary>
    /// Returns o list of all pairs of keys that its transpose define an edge in the directed graph.
    /// </summary>
    /// <exception cref="InvalidOperationException">Undirected graph does not have a transpose.</exception>
    public List<(TKey, TKey)> TransposedEdges => IsDirected
        ? _edges.Select(pair => pair.Key.TransposedTuple).ToList()
        : throw new InvalidOperationException("Undirected graph does not have a transpose.");
    /// <summary>
    /// Returns a list of all vertex keys in the graph.
    /// </summary>
    public List<TKey> Vertices => _vertices.Select(pair => pair.Key).ToList();
    public readonly bool IsDirected;
    public readonly float DefaultWeight;

    /// <summary>
    /// Returns a graph object that is the transpose of this one.
    /// </summary>
    /// <exception cref="InvalidOperationException">Undirected graph does not have a transpose.</exception>
    public Graph<TKey, TValue> Transpose => IsDirected
        ? new Graph<TKey, TValue>(this, true)
        : throw new InvalidOperationException("Undirected graph does not have a transpose.");
    

    #endregion
    
    #region Indexers

    /// <summary>
    /// Access the list of vertices;
    /// </summary>
    /// <param name="key">key of desired vertex to be retrieved,
    /// added or to have its value changed.</param>
    /// <exception cref="ArgumentException">Vertex with given key was not found.</exception>
    /// <exception cref="ArgumentNullException"></exception>
    public TValue this[[NotNull] TKey key] {
        get => ContainsVertex(key) ? _vertices[key] 
            : throw new ArgumentException( "Vertex with given key not found.");
        set { if (value == null) throw new ArgumentNullException(nameof(value));
            if (ContainsVertex(key)) _vertices[key] = value;
            else AddVertex(key, value);
        }
    }

    /// <summary>
    /// Retrieves a copy of an edge form the edge list or null if not found.
    /// </summary>
    /// <param name="pair">Pair of keys of desired edge vertices.</param>
    /// <exception cref="ArgumentNullException"></exception>
    /// <exception cref="ArgumentException">Edge was not found.</exception>
    public Edge<TKey> this[(TKey, TKey) pair] => ContainsEdge(pair)
        ? new Edge<TKey>(_edges[new KeyPair<TKey>(pair)])
        : throw new ArgumentException("Edge was not found.");
    /// <summary>
    /// Access and updates the weight of a given edge,
    /// it will add the edge with the given weight if its not found.
    /// </summary>
    /// <param name="a">Key of first vertex, the source in directed graphs</param>
    /// <param name="b">Key of second vertex, the destination in directed graphs</param>
    /// <exception cref="ArgumentNullException"></exception>
    public float this[[NotNull] TKey a, [NotNull] TKey b] {
        get{
            var pair = new KeyPair<TKey>(a, b);
            return ContainsEdge(pair) ? _edges[pair].Weight : AddEdge(pair).Weight;
        }
        set{
            var pair = new KeyPair<TKey>(a, b);
            if (ContainsEdge(pair)) _edges[pair].Weight = value;
            else AddEdge(pair).Weight = value;
        }
    }

    #endregion

    #region Constructors
    
    /// <summary>
    /// Initializes a new instance of the <see cref="Graph{TKey,TValue}"/> class.
    /// </summary>
    /// <param name="defaultWeight">The weight that is given to every edge with an uninitialized weight.</param>
    /// <param name="isDirected">If true, every edge added will be treated as directed.</param>
    public Graph (float defaultWeight, bool isDirected = false) {
        DefaultWeight = defaultWeight;
        IsDirected = isDirected;
        
        _vertices = new Dictionary<TKey, TValue>();
        _adjacency = new Dictionary<TKey, List<TKey>>();
        _transpose = new Dictionary<TKey, List<TKey>>();
        _edges = isDirected ? 
            new Dictionary<KeyPair<TKey>, Edge<TKey>>(KeyPair<TKey>.DirectedComparer) : 
            new Dictionary<KeyPair<TKey>, Edge<TKey>>(KeyPair<TKey>.UndirectedComparer);
    }
    /// <summary>
    /// Initializes a new instance of the <see cref="Graph{TKey,TValue}"/> class.
    /// </summary>
    /// <param name="isDirected">If true, every edge added will be treated as directed.</param>
    public Graph(bool isDirected) : this(Mathf.Infinity, isDirected) { }
    /// <summary>
    /// Initializes a new instance of the <see cref="Graph{TKey,TValue}"/> class.
    /// </summary>
    public Graph() : this(Mathf.Infinity, false) { }
    private Graph([NotNull] Graph<TKey, TValue> other, bool transpose) : 
        this(other.DefaultWeight, other.IsDirected) {
        if (other == null) throw new ArgumentNullException(nameof(other));
        other.Vertices.ForEach(key => this[key] = other[key]);
        if(transpose) other.Edges.ForEach(pair => this[pair.Item1, pair.Item2] = other[pair].Weight);
        else other.Edges.ForEach(pair => this[pair.Item2, pair.Item1] = other[pair].Weight);
    }
    /// <summary>
    /// Initializes a new instance of the <see cref="Graph{TKey,TValue}"/> class.
    /// </summary>
    /// <param name="other">The other graph object that will be copied from</param>
    public Graph([NotNull] Graph<TKey, TValue> other) : this(other, false) { }


    #endregion

    #region Contains

    /// <summary>
    /// Checks if graph contains a vertex with given key.
    /// </summary>
    /// <param name="vert">Vertex key to be checked.</param>
    /// <returns>True if vertex with key exist in the graph, false otherwise.</returns> 
    /// <exception cref="ArgumentNullException"></exception>
    public bool ContainsVertex([NotNull] TKey vert) {
        if (vert == null) throw new ArgumentNullException(nameof(vert));
        return _vertices.ContainsKey(vert);
    }
    /// <summary>
    /// Checks if edge exists in graph.
    /// </summary>
    /// <param name="from">Key of first vertex, the source in directed graphs</param>
    /// <param name="to">Key of second vertex, the destination in directed graphs</param>
    /// <returns>True if edge exists, false otherwise.</returns>
    /// <exception cref="ArgumentNullException"></exception>
    public bool ContainsEdge([NotNull] TKey from, [NotNull] TKey to) {
        if (from == null) throw new ArgumentNullException(nameof(from));
        if (to == null) throw new ArgumentNullException(nameof(to)); 
        return _edges.ContainsKey(new KeyPair<TKey>(from, to));
    }
    /// <summary>
    /// Checks if edge exists in graph.
    /// </summary>
    /// <param name="pair">Pair of keys of the vertices that define the edge, for a directed graph
    /// the first is the source and the second the destination.</param>
    /// <returns>True if edge exists, false otherwise.</returns>
    /// <exception cref="ArgumentNullException"></exception>
    public bool ContainsEdge([NotNull] (TKey, TKey) pair) => pair.Item1 != null && pair.Item2 != null 
            ? _edges.ContainsKey(new KeyPair<TKey>(pair)) : throw new ArgumentNullException(nameof(pair));
    
    private bool ContainsEdge([NotNull] KeyPair<TKey> pair) => 
        pair != null ? _edges.ContainsKey(pair) : throw new ArgumentNullException(nameof(pair));
    
    private bool ContainsVertex(params TKey[] verts) => verts.All(v => _vertices.ContainsKey(v));
    private bool ContainsEdge(params KeyPair<TKey>[] pairs) => pairs.All(key => _edges.ContainsKey(key));
    private bool ContainsEdge(params Edge<TKey>[] edges) => edges.All(edge => _edges.ContainsKey(edge.Vertices));  

    #endregion

    #region Get
    
    /// <summary>
    /// Retrieves a copy of the adjacency list of a given vertex.
    /// </summary>
    /// <param name="key">Key of the desired adjacency list's vertex</param>
    /// <returns>A list containing the keys of all vertices in this graph
    /// that has an edge with the required vertex.</returns>
    /// <exception cref="ArgumentException">Vertex with given key was not found</exception>
    public IEnumerable<TKey> GetAdjacency([NotNull] TKey key) => ContainsVertex(key)
        ? new List<TKey>(_adjacency[key])
        : throw new ArgumentException("Vertex with given key was not found");
    
    /// <summary>
    /// Retrieves a copy of the adjacency list of a given vertex.
    /// </summary>
    /// <param name="key">Key of the desired adjacency list's vertex</param>
    /// <returns>A list containing the keys of all vertices in this graph
    /// that has an edge with the required vertex.</returns>
    /// <exception cref="ArgumentException">Vertex with given key was not found.</exception>
    /// <exception cref="InvalidOperationException">Undirected graph does not have a transpose.</exception>
    public IEnumerable<TKey> GetTranspose([NotNull] TKey key) => ContainsVertex(key)
        ? IsDirected ? new List<TKey>(_transpose[key])
            : throw new InvalidOperationException("Undirected graph does not have a transpose.")
        : throw new ArgumentException("Vertex with given key was not found.");

    #endregion
    
    #region Add
    private bool AddVertex ([NotNull] TKey vertKey, [NotNull] TValue vertValue) {
        if (ContainsVertex(vertKey)) return false;
        
        _vertices.Add(vertKey, vertValue);
        _adjacency.Add(vertKey, new List<TKey>());
        if(IsDirected) _transpose.Add(vertKey, new List<TKey>());
        return true;
    }
    private Edge<TKey> AddEdge ([NotNull] KeyPair<TKey> pair) {
        var (from, to) = (pair[0], pair[1]);
        
        //Avoid self loops on undirected graphs
        if (!IsDirected && from.Equals(to)) 
            throw new ArgumentException("Undirected graph cannot have self loops");
        
        //Check if the vertices really exist in the graph
        if (!ContainsVertex(from, to))
            throw new ArgumentException("Cannot create an edge with vertices not in the graph.");

        //Create the edge
        var edge = new Edge<TKey>(from, to, DefaultWeight);
        _adjacency[from].Add(to);
        if( !IsDirected ) _adjacency[to].Add(from);
        else _transpose[to].Add(from);
        
        _edges.Add(edge.Vertices, edge);
        
        return edge;
    }

    #endregion

    #region Remove

    /// <summary>
    /// Removes a vertex from the graph, and every edge containing it.
    /// </summary>
    /// <param name="vert">key of the vertex to be removed.</param>
    /// <returns>True if the vertex was successfully removed, false otherwise</returns>
    public bool RemoveVertex ([NotNull] TKey vert) {
        if (!ContainsVertex(vert)) return false;

        _vertices.Remove(vert);
        new List<TKey>(_adjacency[vert]).ForEach(adj => RemoveEdge((vert, adj)));
        _adjacency.Remove(vert);

        if (!IsDirected) return true;
        
        new List<TKey>(_transpose[vert]).ForEach(adj => RemoveEdge((adj, vert)));
        _transpose.Remove(vert);
        
        return true;
    }

    /// <summary>
    /// Remove an edge from the graph
    /// </summary>
    /// <param name="pair">Pair of key that defines the edge, for a directed graph
    /// the first is the source and the second the destination.</param>
    /// <returns>True if the edge was successfully removed, false otherwise</returns>
    public bool RemoveEdge([NotNull] (TKey, TKey) pair) {
        if(!ContainsEdge(pair)) return false;

        var (from, to) = pair;
        _adjacency[from].Remove(to);
        if (!IsDirected) _adjacency[to].Remove(from);

        _edges.Remove(new KeyPair<TKey>(pair));
        
        return true;
    }

    #endregion

    #region Search

    private enum SearchColor {
        White, Gray, Black
    }

    /// <summary>
    /// Performs an breadth-first search in the graph.
    /// </summary>
    /// <param name="seed">Vertex which will be used to start the search</param>
    /// <param name="transpose">Decides whether the edges of the bfs tree will point from parent to child (true)
    /// or child to parent (false)</param>
    /// <returns>A Breadth-First Tree generated in the search populated with every vertex reachable by the seed
    /// containing its shortest path distance from the seed</returns>
    public Graph<TKey, int> BreadthFirstSearch([NotNull] TKey seed,  bool transpose = true) => 
        BreadthFirstSearch<int>(seed, 0, (du, v) => du+1, transpose);
    /// <summary>
    /// Performs an breadth-first search in the graph.
    /// </summary>
    /// <param name="seed">Vertex which will be used to start the search</param>
    /// <param name="seedData">The data that will be contained in the root of bfs tree returned</param>
    /// <param name="dataSelector">Function that will be used to calculate the data of every vertex in
    /// the bfs tree given the data of its parent vertex and its key</param>
    /// <param name="transpose">Decides whether the edges of the bfs tree will point from parent to child (true)
    /// or child to parent (false)</param>
    /// <typeparam name="TOut">Type of the data value that will be stored in the bfs tree's vertices</typeparam>
    /// <returns>A Breadth-First Tree generated in the search populated with every vertex reachable by the seed
    /// and containing the data calculated by the dataSelector function</returns>
    /// <exception cref="ArgumentNullException"></exception>
    public Graph<TKey, TOut> BreadthFirstSearch<TOut>([NotNull] TKey seed, [NotNull] TOut seedData, [NotNull] Func<TOut, TKey, TOut> dataSelector,  bool transpose = true) {
        if (seedData == null) throw new ArgumentNullException(nameof(seedData));
        if (dataSelector == null) throw new ArgumentNullException(nameof(dataSelector));
        if (!ContainsVertex(seed)) return null;

        var colors = _vertices.ToDictionary(pair => pair.Key, pair => SearchColor.White);
        var parentTree = new Graph<TKey, TOut>(DefaultWeight, true);
        
        colors[seed] = SearchColor.Gray;
        parentTree[seed] = seedData;

        var queue = new Queue<TKey>(new []{seed});
        while (queue.Count > 0) {
            var u = queue.Dequeue();
            _adjacency[u].ForEach(v => {
                if (colors[v] != SearchColor.White) return;
                colors[v] = SearchColor.Gray;

                
                parentTree[v] = dataSelector(parentTree[u], v);
                if (transpose) parentTree[v, u] = this[u, v];
                else parentTree[u, v] = this[u, v];
                queue.Enqueue(v);
            });
            colors[u] = SearchColor.Black;
        }

        return parentTree;
    }


    public Graph<TKey, (int, int)> DepthFirstSearch(bool transpose = true) =>
        DepthFirstSearch(( v, t) => t, (v,d, t) => t, transpose);
    
    /// <summary>
    /// Performs an breadth-first search in the graph.
    /// </summary>
    /// <param name="discover">Function that will be used to calculate the data of every vertex when its
    /// first discovered given its key and the current timestamp</param>
    /// <param name="finish">Function that will be used to calculate the data of every vertex when its
    /// finished given its key, the result of its discover function and the current timestamp</param>
    /// <param name="transpose">Decides whether the edges of the bfs tree will point from parent to child (true)
    /// or child to parent (false)</param>
    /// <typeparam name="TOut1">Type of the data value that will be calculated in the discover function</typeparam>
    /// <typeparam name="TOut2">Type of the data value that will be calculated in the finish function</typeparam>
    /// <returns>A Graph object with all Depth-First Trees generated in the search with its vertices
    /// containing the data calculated by the dataSelector function</returns>
    public Graph<TKey, (TOut1, TOut2)> DepthFirstSearch<TOut1, TOut2>([NotNull] Func<TKey, int, TOut1> discover, 
        [NotNull] Func<TKey, TOut1, int, TOut2> finish, bool transpose = true) {
        
        var colors = _vertices.ToDictionary(pair => pair.Key, pair => SearchColor.White);
        var parentTree = new Graph<TKey, (TOut1, TOut2)>(DefaultWeight, true);
        var time = 0;
        
        //TODO: Edge categorization
        void Visit(TKey u) {
            if (colors[u] != SearchColor.White) return;
            parentTree[u] = default;
            var d = discover(u, ++time);
            colors[u] = SearchColor.Gray;
            foreach (var v in _adjacency[u].Where(v => colors[v] == SearchColor.White)) {
                parentTree[v] = default;
                if (transpose) parentTree[v, u] = this[u, v];
                else parentTree[u, v] = this[u, v];
                Visit(v);
            }

            colors[u] = SearchColor.Black;
            var f = finish(u, d, ++time);
            parentTree[u] = (d, f);
        }
        
        Vertices.ForEach(Visit);

        return parentTree;
    }
    #endregion
    
    #region Print

    public override string ToString () => "Directed Graph: " + IsDirected + 
                                          "\nDefault Weight Value: " + DefaultWeight + 
                                          "\nEdges:\n" + PrintAdjacency() +
                                          "\nVertices:\n" + PrintVertices();
    private string PrintVertex(TKey key) => key + ": " + this[key];
    
    private string PrintVertices () =>
        _vertices.Aggregate("", (str, pair) => str + "\t[ " + PrintVertex(pair.Key) + " ]\n");
    private string PrintAdjacency () =>
        _vertices.Aggregate("", (str, pair) => str + "\t " + PrintEdges(pair.Key) + "\n");
    private string PrintEdges(TKey key) => GetAdjacency(key).Aggregate(key + " -> ",
        (str, other) => str + " " + this[(key, other)]);

    #endregion
}

}