using System;
using System.Collections.Generic;
using System.Linq;
using JetBrains.Annotations;

namespace Utils.ProcGenUtils.GraphModel {

/// <summary>
/// Implements the disjoint sets data structure.
/// </summary>
/// <typeparam name="T">Type of the objects in the sets</typeparam>
public class DisjointSets<T> {
	private class Node {
		[NotNull] public T Parent;
		public int Rank;

		public Node(T parent, int rank) {
			Parent = parent;
			Rank = rank;
		}
	}

	private readonly Dictionary<T, Node> _nodes;
	
	/// <summary>
	/// Get: Returns the representative of the set of x. (Equivalent to FindSet)
	/// Set: Performs a union of x and the value assigned. (Equivalent to Union)
	/// </summary>
	public T this[T x] {
		get => FindSet(x);
		set => Union(x, value);
	}
	
	/// <summary>
	/// Initializes a new instance of the <see cref="DisjointSets{T}"/> class.
	/// </summary>
	/// <param name="items">List of objects to initialize the structure</param>
	/// <exception cref="ArgumentNullException"></exception>
	public DisjointSets([NotNull] IEnumerable<T> items) {
		if (items == null) throw new ArgumentNullException(nameof(items));
		_nodes = new Dictionary<T, Node>();
		items.ToList().ForEach(MakeSet);
	}

	/// <summary>
	/// Adds a new set containing the object.
	/// </summary>
	/// <param name="x">Object representative of its set</param>
	/// <exception cref="ArgumentNullException"></exception>
	/// <exception cref="ArgumentException">"Object is already a set."</exception>
	public void MakeSet([NotNull] T x) {
		if (x == null) throw new ArgumentNullException(nameof(x));
		if (_nodes.ContainsKey(x)) 
			throw new ArgumentException("Object is already a set.");
		_nodes.Add(x, new Node(x, 0));
	}

	/// <summary>
	/// Joins the sets of x and y.
	/// </summary>
	/// <exception cref="ArgumentNullException"></exception>
	/// <exception cref="ArgumentException">"Object is not a set."</exception>
	public void Union([NotNull] T x, [NotNull] T y) {
		if (x == null) throw new ArgumentNullException(nameof(x));
		if (y == null) throw new ArgumentNullException(nameof(y));
		if (!_nodes.ContainsKey(x) || !_nodes.ContainsKey(y)) 
			throw new ArgumentException("Object is not a set.");
		Link(this[x], this[y]);
	}

	private void Link(T x, T y) {
		var rankX = _nodes[x].Rank;
		var rankY = _nodes[y].Rank;
		if (rankX > rankY)
			_nodes[y].Parent = x;
		else {
			_nodes[x].Parent = y;
			if (rankX == rankY) _nodes[y].Rank++;
		}
	}

	/// <summary>
	/// Discovers the representative object of x's set
	/// </summary>
	/// <returns>x's set representative</returns>
	/// <exception cref="ArgumentNullException"></exception>
	/// <exception cref="ArgumentException">"Object is not a set."</exception>
	public T FindSet([NotNull] T x) {
		if (x == null) throw new ArgumentNullException(nameof(x));
		if (!_nodes.ContainsKey(x)) throw new ArgumentException("Object is not a set.");
		
		var parent = _nodes[x].Parent;
		if (!x.Equals(parent)) 
			_nodes[x].Parent = FindSet(parent);
		return parent;
	}

}

}