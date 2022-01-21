using System;
using System.Collections.Generic;
using System.Linq;
using JetBrains.Annotations;

namespace Utils.ProcGenUtils.DataStructures {

/// <summary>
/// An implementation of a Fibonacci Heap (Minimum)
/// </summary>
/// <typeparam name="TObj">Object that will be contained in the node</typeparam>
/// <typeparam name="TKey">Type of the key that will be used to compare nodes</typeparam>
public class FibonacciHeap<TObj, TKey> where TKey : IComparable  {
	private class HeapNode {
		public readonly TObj NodeObject;
		public TKey Key;
		public HeapNode Parent;
		public bool Mark;
		
		public readonly LinkedList<HeapNode> Children;
		public int Degree => Children.Count;

		private LinkedListNode<HeapNode> _self;
		public HeapNode Right => _self?.NextCircular()?.Value;
		public HeapNode Left => _self?.PreviousCircular()?.Value;

		public HeapNode () {
			Children = new LinkedList<HeapNode>();
		}
		public HeapNode (TObj nodeObject, TKey key) : this() {
			NodeObject = nodeObject;
			Key = key;
		}

		public void AddChild(HeapNode node) {
			node.Parent = this;
			node._self = Children.AddLast(node);
		}
		public void RemoveChild(HeapNode node) {
			node.Parent = null;
			Children.Remove(node._self);
			node._self = null;
		}

		public override string ToString () => (Parent.Parent == null ? " " : "") + (Degree > 0
			? Children.Aggregate(Key + "(", (s, child) => s + " " + child, s => s + ")")
			: Key + " ");
	}
	
	private readonly HeapNode _root;
	private LinkedList<HeapNode> RootList => _root.Children;
	private HeapNode _minNode;

	private readonly Dictionary<TObj, HeapNode> _nodes;
	public int Count => _nodes.Count;
	public TObj Min => _minNode != null ? _minNode.NodeObject : default;

	/// <summary>
	/// Initializes a new instance of the <see cref="DisjointSets{T}"/> class.
	/// </summary>
	public FibonacciHeap ( ) {
		_root = new HeapNode();
		_nodes = new Dictionary<TObj, HeapNode>();
	}

	private void AddToRootList(HeapNode elem) => _root.AddChild(elem);
	private void RemoveFromRootList(HeapNode elem) => _root.RemoveChild(elem);

	/// <summary>
	/// Checks if object is in the heap
	/// </summary>
	/// <returns>True if it is, false if not</returns>
	/// <exception cref="ArgumentNullException"></exception>
	public bool Contains([NotNull] TObj obj) => 
		obj == null ? throw new ArgumentNullException(nameof(obj)) : _nodes.ContainsKey(obj);

	#region Insert/Decrease/Remove

	/// <summary>
	/// Inserts a node with the obj with the given key, or if obj is already
	/// in the heap, decreases its key
	/// </summary>
	/// <param name="obj">Object to be inserted or have its key decreased in the heap</param>
	/// <exception cref="ArgumentNullException"></exception>
	/// <exception cref="ArgumentException">Object not present in the heap.</exception>
	/// <exception cref="InvalidOperationException">New key is greater than previous key.</exception>
	[NotNull]
	public TKey this[[NotNull] TObj obj] {
		get => !Contains(obj) ? throw new ArgumentException("Object not present in the heap.") : _nodes[obj].Key;
		set {
			if (value == null) throw new ArgumentNullException(nameof(value));
			if (!Contains(obj)) Insert(obj, value);
			else {
				if (value.CompareTo(_nodes[obj].Key) > 0)
					throw new InvalidOperationException("New key is greater than previous key.");
				DecreaseKey(obj, value);
			}
		}
	}
	
	private void Insert (TObj obj, TKey key) {
		var element = new HeapNode(obj, key);
		_root.AddChild(element);
		if(_minNode == null || _minNode.Key.CompareTo(key) > 0)
			_minNode = element;
		_nodes.Add(obj, element);
	}

	private void DecreaseKey(TObj obj, TKey key) {
		var x = _nodes[obj];

		x.Key = key;
		var y = x.Parent;
		if (y != _root && x.Key.CompareTo(y.Key) < 0) {
			HeapCut(x, y);
			HeapCascadingCut(y);
		}

		if (key.CompareTo(_minNode.Key) < 0) _minNode = x;

	}
	private void HeapCut(HeapNode x, HeapNode y) {
		y.RemoveChild(x);
		AddToRootList(x);
		x.Mark = false;
	}
	private void HeapCascadingCut(HeapNode x) {
		var y = x.Parent;
		while (y != _root) {
			if (x.Mark) {
				HeapCut(x, y);
				x = y;
				y = x.Parent;
				continue;
			}
			
			x.Mark = true;
			break;
		}
	}

	/// <summary>
	/// Removes a node from the heap
	/// </summary>
	/// <exception cref="ArgumentNullException"></exception>
	/// <exception cref="ArgumentException">Object not present in the heap.</exception>
	public void Remove([NotNull] TObj obj) {
		if (!Contains(obj)) throw new ArgumentException("Object not present in the heap.");
		
		var x = _nodes[obj];
		
		var y = x.Parent;
		if (y != _root) {
			HeapCut(x, y);
			HeapCascadingCut(y);
		}

		_minNode = x;
		ExtractMin();
	}
	
	#endregion
	
	#region ExtractMin

	/// <summary>
	/// Extracts the node with minimum key
	/// </summary>
	/// <returns>The object that held the minimum key in the heap</returns>
	/// <exception cref="InvalidOperationException">Cannot extract minimum element from an empty heap.</exception>
	public TObj ExtractMin () {
		var min = _minNode ??
		          throw new InvalidOperationException("Cannot extract minimum element from an empty heap.");
		foreach (var child in min.Children) 
			AddToRootList(child);
		
		var right = min.Right;
		RemoveFromRootList(min);

		if (RootList.Count == 0) _minNode = null;
		else {
			_minNode = right;
			ConsolidateHeap();
		}
		
		_nodes.Remove(min.NodeObject);
		return min.NodeObject;
	}
	private void ConsolidateHeap () {
		var upperBound = (int) Math.Floor(Math.Log(Count, 2f)) * 2 + 1;
		var degrees = new HeapNode[upperBound];

		var node = _minNode;
		var last = _minNode.Left;
		while (true) {
			var x = node;
			var right = node.Right;
			var d = x.Degree;
			while (degrees[d] != null) {
				var y = degrees[d];
				if (x.Key.CompareTo(y.Key) >= 0) (x, y) = (y, x);
				HeapLink(x, y);
				degrees[d] = null;
				d++;
			}

			degrees[d] = x;

			if(node == last) break;
			node = right;

		}

		_minNode = RootList.First.Value;
		foreach (var elem in RootList) 
			_minNode = elem.Key.CompareTo(_minNode.Key) < 0 ? elem : _minNode;
	}
	private void HeapLink(HeapNode x, HeapNode y) {
		RemoveFromRootList(y);
		x.AddChild(y);
		y.Mark = false;
	}

	#endregion

	public override string ToString () => RootList.Aggregate("", (s, child) => s + child);
}

}