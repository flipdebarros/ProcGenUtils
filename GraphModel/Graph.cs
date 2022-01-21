using System;
using System.Collections.Generic;
using System.Linq;
using JetBrains.Annotations;

namespace Utils.ProcGenUtils.GraphModel {
/// <summary>
/// A container that implements the mathematical model of a generic weighted graph,
/// it can be directed or undirected
/// </summary>
/// <typeparam name="TKey">Type that will be used to index the vertices.</typeparam>
/// <typeparam name="TValue">Type that vertices will hold as value</typeparam>
public class Graph<TKey, TValue> {
    private readonly Dictionary<TKey, TValue> _vertices;
    private readonly Dictionary<TKey, List<TKey>> _adjacency;
    private readonly Dictionary<TKey, List<TKey>> _transpose;
    private readonly Dictionary<KeyPair<TKey>, Edge<TKey>>_edges;

    private int _nNegativeEdges = 0;

    #region Public Properties

    /// <summary>
    /// Returns o list of all pairs of keys that define an edge in the graph.
    /// </summary>
    public List<(TKey, TKey)> Edges => _edges.Select(pair => pair.Key.Tuple).ToList();
    public int EdgeCount => _edges.Count;
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
    public int VertexCount => _vertices.Count;
    public readonly bool IsDirected;
    public bool HasNegativeWeights => _nNegativeEdges > 0;
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
        get => ContainsEdge(a, b) ? _edges[new KeyPair<TKey>(a, b)].Weight : AddEdge((a, b)).Weight;
        set {
            if (ContainsEdge(a, b)) {
                var edge = _edges[new KeyPair<TKey>(a, b)];
                if (value < 0f && edge.Weight > 0f) _nNegativeEdges++;
                else if (value > 0f && edge.Weight < 0f) _nNegativeEdges--;
                edge.Weight = value;
            }
            else AddEdge((a, b)).Weight = value; }
    }
    internal float this[[NotNull] TKey a, [NotNull] TKey b, EdgeType type] {
        set{
            if (ContainsEdge(a, b)) throw new ArgumentException("An existing edge should not be changed.");
            var edge = AddEdge((a,b));
            edge.Weight = value;
            edge.Type = type;
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
    public Graph(bool isDirected) : this(float.PositiveInfinity, isDirected) { }
    /// <summary>
    /// Initializes a new instance of the <see cref="Graph{TKey,TValue}"/> class.
    /// </summary>
    public Graph() : this(float.PositiveInfinity) { }
    private Graph([NotNull] Graph<TKey, TValue> other, bool transpose) : 
        this(other.DefaultWeight, other.IsDirected) {
        if (other == null) throw new ArgumentNullException(nameof(other));
        other.Vertices.ForEach(key => AddVertex(key, other[key]));
        if (!transpose) other.Edges.ForEach(pair => AddEdge(other[pair]));
        else other.Edges.ForEach(pair =>
                AddEdge(other[pair].TransposedTuple, other[pair].Weight, other[pair].Type));
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
    public bool ContainsEdge((TKey, TKey) pair) => pair.Item1 != null && pair.Item2 != null 
            ? _edges.ContainsKey(new KeyPair<TKey>(pair)) : throw new ArgumentNullException(nameof(pair));

    private bool ContainsVertex(params TKey[] verts) => verts.All(v => _vertices.ContainsKey(v));

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
    private void AddVertex([NotNull] TKey vertKey, [NotNull] TValue vertValue) {
        if (ContainsVertex(vertKey)) return;

        _vertices.Add(vertKey, vertValue);
        _adjacency.Add(vertKey, new List<TKey>());
        if(IsDirected) _transpose.Add(vertKey, new List<TKey>());
    }

    private void AddEdge([NotNull] Edge<TKey> edge) => AddEdge(edge.Tuple, edge.Weight, edge.Type);
    private Edge<TKey> AddEdge((TKey, TKey) pair) => AddEdge(pair, DefaultWeight);
    private Edge<TKey> AddEdge ((TKey, TKey) pair, float weight, EdgeType type = EdgeType.Null) {
        var (from, to) = pair;

        //Avoid self loops on undirected graphs
        if (!IsDirected && from.Equals(to)) 
            throw new ArgumentException("Undirected graph cannot have self loops");
        
        //Check if the vertices really exist in the graph
        if (!ContainsVertex(from, to))
            throw new ArgumentException("Cannot create an edge with vertices not in the graph.");

        if (weight < 0f) _nNegativeEdges++;

        //Create the edge
        var edge = new Edge<TKey>(pair, weight, type);
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
    public bool RemoveEdge((TKey, TKey) pair) {
        if(!ContainsEdge(pair)) return false;

        var (from, to) = pair;
        _adjacency[from].Remove(to);
        if (!IsDirected) _adjacency[to].Remove(from);

        _edges.Remove(new KeyPair<TKey>(pair));
        
        return true;
    }

    #endregion

    private enum SearchColor {
        White, Gray, Black
    }

    #region Breadth-First

    /// <summary>
    /// Calculates the simple shortest path from the seed to every vertex reachable by it.
    /// </summary>
    /// <param name="seed">Seed vertex key</param>
    /// <returns>A pair of dictionaries in which, the first contains the distance from v to the seed
    /// and the second contains the vertex that its edge with v will lead v to s the fastest</returns>
    public (Dictionary<TKey, int>, Dictionary<TKey, TKey>) BFSShortestPath([NotNull] TKey seed) {
        var dist = new Dictionary<TKey, int>{{seed, 0}};
        var pi = new Dictionary<TKey, TKey>();
        
        BreadthFirstSearch(seed, (u, v) => { dist.Add(v, dist[u]+1); pi.Add(v, u); });
        return (dist, pi);
    }

    /// <summary>
    /// Performs an breadth-first search in the graph.
    /// </summary>
    /// <param name="seed">Vertex which will be used to start the search</param>
    /// <param name="discover">Function that will be called when an edge finds a vertex that
    /// hasn't already been discovered, it receives the current vertex and the one whose edge found it</param>
    /// <exception cref="ArgumentNullException"></exception>
    /// <exception cref="ArgumentException">Seed not in the graph.</exception>
    public void BreadthFirstSearch([NotNull] TKey seed, [NotNull] Action<TKey, TKey> discover) =>
        BreadthFirstSearch(seed, discover, (u, v) => { });

    /// <summary>
    /// Performs an breadth-first search in the graph.
    /// </summary>
    /// <param name="seed">Vertex which will be used to start the search</param>
    /// <param name="discover">Function that will be called when an edge finds a vertex that
    /// hasn't already been discovered, it receives the current vertex and the one whose edge found it</param>
    /// <param name="revisit">Function that will be called when an edge finds a vertex that
    /// has already been discovered, it receives the current vertex and the one whose edge found it</param>
    /// <exception cref="ArgumentNullException"></exception>
    /// <exception cref="ArgumentException">Seed not in the graph.</exception>
    public void BreadthFirstSearch([NotNull] TKey seed, [NotNull] Action<TKey, TKey> discover,
        [NotNull] Action<TKey, TKey> revisit) {
        if (discover == null) throw new ArgumentNullException(nameof(discover));
        if (revisit == null) throw new ArgumentNullException(nameof(revisit));
        if (!ContainsVertex(seed)) throw new ArgumentException("Seed not in the graph.");

        var colors = _vertices.ToDictionary(pair => pair.Key, pair => SearchColor.White);

        colors[seed] = SearchColor.Gray;

        var queue = new Queue<TKey>(new []{seed});
        while (queue.Count > 0) {
            var u = queue.Dequeue();
            _adjacency[u].ForEach(v => {
                if (colors[v] != SearchColor.White) {
                    revisit(u, v);
                    return;
                }
                
                colors[v] = SearchColor.Gray;
                discover(u, v);
                queue.Enqueue(v);
            });
            colors[u] = SearchColor.Black;
        }
    }

    /// <summary>
    /// Runs a Breadth-first search in the graph to construct a breadth-first tree.
    /// </summary>
    /// <param name="seed">The seed vertex used to start the search</param>
    /// <param name="seedData">The value that the seed vertex will hold</param>
    /// <param name="dataSelector">Function that defines each vertex value,
    /// given its parent value and the current vertex key</param>
    /// <param name="addCrossEdges">Decides whether or not to add the remaining edges of the
    /// source graph that don't make up the tree, marked with the type "Cross"</param>
    /// <typeparam name="TOut">Type of the value that the vertices will hold</typeparam>
    /// <returns>A graph object containing the bfs tree resulting from the search</returns>
    /// <exception cref="ArgumentNullException"></exception>
    /// <exception cref="ArgumentException">Seed not in the graph.</exception>
    public Graph<TKey, TOut> BreadthFirstTree<TOut>([NotNull] TKey seed, [NotNull] TOut seedData,
        [NotNull] Func<TOut, TKey, TOut> dataSelector, bool addCrossEdges = false) {
        
        var tree = new Graph<TKey, TOut>(DefaultWeight, true) { [seed] = seedData };
        
        void Discover(TKey u, TKey v) {
            tree[v] = dataSelector(tree[u], v);
            tree[u, v, EdgeType.Tree] = this[u, v];
        }

        void Revisit(TKey u, TKey v) {
            if (!IsDirected && tree.ContainsEdge(v, u)) return;
            tree[u, v, EdgeType.Cross] = this[u, v];
        }

        if(addCrossEdges) BreadthFirstSearch(seed, Discover, Revisit);
        else BreadthFirstSearch(seed, Discover);

        return tree;
    }

    #endregion

    #region Depth-First

    /// <summary>
    /// Performs an depth-first search in the graph.
    /// </summary>
    /// <param name="discover">Function that will be used to calculate the data of every vertex when its
    /// first discovered given its key and the current timestamp</param>
    /// <param name="finish">Function that will be used to calculate the data of every vertex when its
    /// finished given its key and the current timestamp</param>
    /// <exception cref="ArgumentNullException"></exception>
    public void DepthFirstSearch([NotNull] Action<TKey> discover, [NotNull] Action<TKey> finish) =>
        DepthFirstSearch(Vertices, discover, finish);
    
    /// <summary>
    /// Performs an depth-first search in the graph.
    /// </summary>
    /// <param name="vertices">List of vertices that will be iterated over to perform the search</param>
    /// <param name="discover">Function that will be used to calculate the data of every vertex when its
    /// first discovered given its key and the current timestamp</param>
    /// <param name="finish">Function that will be used to calculate the data of every vertex when its
    /// finished given its key and the current timestamp</param>
    /// <exception cref="ArgumentNullException"></exception>
    public void DepthFirstSearch([NotNull] List<TKey> vertices, [NotNull] Action<TKey> discover, [NotNull] Action<TKey> finish) => 
        DepthFirstSearch(vertices, discover, (u, v) => {}, 
        (u, v) => {}, (u, v) => {}, finish);

    /// <summary>
    /// Performs an depth-first search in the graph.
    /// </summary>
    /// <param name="vertices">List of vertices that will be iterated over to perform the search</param>
    /// <param name="discover">Function that will be called when a vertex is first discovered by the algorithm</param>
    /// <param name="explore">Function that will be called when a vertex is first explored by the algorithm,
    /// before its discovered</param>
    /// <param name="finish">Function that will be called when a vertex is marked as finished by the algorithm</param>
    /// <exception cref="ArgumentNullException"></exception>
    public void DepthFirstSearch([NotNull] List<TKey> vertices, [NotNull] Action<TKey> discover,
        [NotNull] Action<TKey, TKey> explore, [NotNull] Action<TKey> finish) =>
        DepthFirstSearch(vertices, discover, explore, (u, v) => { }, (u, v) => { }, finish);
    
    /// <summary>
    /// Performs an depth-first search in the graph.
    /// </summary>
    /// <param name="vertices">List of vertices that will be iterated over to perform the search</param>
    /// <param name="discover">Function that will be called when a vertex is first discovered by the algorithm</param>
    /// <param name="explore">Function that will be called when a vertex is first explored by the algorithm,
    /// before its discovered</param>
    /// <param name="visit">Function that will be called when a vertex is visited by the algorithm,
    /// after it was discovered but before its finished</param>
    /// <param name="revisit">Function that will be called when a vertex is visited by the algorithm,
    /// after its finished</param>
    /// <param name="finish">Function that will be called when a vertex is marked as finished by the algorithm</param>
    /// <exception cref="ArgumentNullException"></exception>
    public void DepthFirstSearch([NotNull] List<TKey> vertices, [NotNull] Action<TKey> discover, 
        [NotNull] Action<TKey, TKey> explore, [NotNull] Action<TKey, TKey> visit, 
        [NotNull] Action<TKey, TKey> revisit, [NotNull] Action<TKey> finish){
        if (vertices == null) throw new ArgumentNullException(nameof(vertices));
        if (discover == null) throw new ArgumentNullException(nameof(discover));
        if (explore == null) throw new ArgumentNullException(nameof(explore));
        if (visit == null) throw new ArgumentNullException(nameof(visit));
        if (revisit == null) throw new ArgumentNullException(nameof(revisit));
        if (finish == null) throw new ArgumentNullException(nameof(finish));

        var colors = _vertices.ToDictionary(pair => pair.Key, pair => SearchColor.White);

        void Visit(TKey u) {
            if (colors[u] != SearchColor.White) return;

            discover(u);
            colors[u] = SearchColor.Gray;
            foreach (var v in _adjacency[u]) {
                if (!vertices.Contains(v)) continue;
                switch (colors[v]) {
                    case SearchColor.White:
                        explore(u, v);
                        Visit(v);
                        break;
                    case SearchColor.Gray:
                        visit(u, v);
                        break;
                    case SearchColor.Black:
                        revisit(u, v);
                        break;
                }
            }

            colors[u] = SearchColor.Black;
            finish(u);
        }
        
        vertices.ForEach(Visit);
    }

    /// <summary>
    /// Runs a Depth-first search in the graph to construct a breadth-first tree.
    /// </summary>
    /// <param name="allEdges">Decides whether or not to add the remaining edges of the
    /// source graph that don't make up the tree, marked with the types "Cross", "Forward" or "Back"</param>
    /// <returns>A graph object containing the dfs tree resulting from the search</returns>
    /// <exception cref="ArgumentNullException"></exception>
    public Graph<TKey, (int, int)> DepthFirstTree(bool allEdges = false) =>
        DepthFirstTree((v, t) => t, (v, d, t) => (d, t), allEdges);
    
    /// <summary>
    /// Runs a Depth-first search in the graph to construct a breadth-first tree.
    /// </summary>
    /// <param name="discover">Function that will be called when a new vertex is discovered by the search,
    /// it calculates a value that will be passed to the finish function given the current vertex key and timestamp</param>
    /// <param name="finish">Function that will be called when a vertex is marked as finished by the search,
    /// it calculates a value that will be stored in the vertex given the current vertex key, the result of
    /// the discover function and the current timestamp</param>
    /// <param name="allEdges">Decides whether or not to add the remaining edges of the
    /// source graph that don't make up the tree, marked with the types "Cross", "Forward" or "Back"</param>
    /// <typeparam name="TDisc">Type of return value from discover function</typeparam>
    /// <typeparam name="TOut">Type of the value that the vertices will hold</typeparam>
    /// <returns>A graph object containing the dfs tree resulting from the search</returns>
    /// <exception cref="ArgumentNullException"></exception>
    public Graph<TKey, TOut> DepthFirstTree<TDisc, TOut> (
        [NotNull] Func<TKey, int, TDisc> discover, 
        [NotNull] Func<TKey, TDisc, int, TOut> finish, bool allEdges = false) {
        
        var tree = new Graph<TKey, TOut>(DefaultWeight, true);
        Vertices.ForEach(v => tree[v] = default);
        var discoverTime = _vertices.ToDictionary(pair => pair.Key, pair => -1);
        var time = 0;

        void Discover(TKey u) {
            discoverTime[u] = ++time;
        }

        void Explore(TKey u, TKey v) {
            tree[u, v, EdgeType.Tree] = this[u, v];
        }

        void Visit(TKey u, TKey v) {
            if (!IsDirected && tree.ContainsEdge(v, u)) return;
            tree[u, v, EdgeType.Back] = this[u, v];
        }

        void Revisit(TKey u, TKey v) {
            if (!IsDirected && tree.ContainsEdge(v, u)) return;
            if (discoverTime[u] < discoverTime[v]) tree[u, v, EdgeType.Forward] = this[u, v];
            else tree[u, v, EdgeType.Cross] = this[u, v];
        }

        void Finish(TKey u) {
            tree[u] = finish(u, discover(u, discoverTime[u]), ++time);
        }
        
        if(allEdges) DepthFirstSearch(Vertices, Discover, Explore, Visit, Revisit, Finish);
        else DepthFirstSearch(Vertices, Discover, Explore, (u, v) => { }, (u, v) => { } , Finish);
        
        return tree;
    }

    #endregion

    #region Print

    public override string ToString () => "Directed Graph: " + IsDirected + 
                                          "\nDefault Weight Value: " + DefaultWeight +
                                          (VertexCount == 0 ? "\nEmpty Graph" :
                                          "\nEdges:\n" + PrintAdjacency() +
                                          "\nVertices:\n" + PrintVertices());
    private string PrintVertex(TKey key) => key + ": " + this[key];
    
    private string PrintVertices () =>
        _vertices.Aggregate("", (str, pair) => str + "\t[ " + PrintVertex(pair.Key) + " ]\n");
    private string PrintAdjacency () =>
        _vertices.Aggregate("", (str, pair) => str + "\t " + PrintEdges(pair.Key) + "\n");
    private string PrintEdges(TKey key) => GetAdjacency(key).Aggregate(key + " -> ",
        (str, other) => str + " /" + this[(key, other)] + "/");

    #endregion
}

}