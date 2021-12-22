using System;
using JetBrains.Annotations;
using UnityEngine;

namespace Utils.ProcGenUtils.GraphModel {

public enum EdgeType {
	Tree, Back, Forward, Cross, Null
}

public class Edge<TKey> {
	[NotNull] internal readonly KeyPair<TKey> Vertices;
	public (TKey, TKey) Tuple => Vertices.Tuple;
	public (TKey, TKey) TransposedTuple => Vertices.TransposedTuple;
	public float Weight;
	public EdgeType Type { get; internal set; }

	internal Edge(Edge<TKey> other) : this(other.Tuple, other.Weight, other.Type) { }
	internal Edge ([NotNull] TKey vKey, [NotNull] TKey uKey, float weight, EdgeType type = EdgeType.Null) : this((vKey, uKey), weight) { }
	internal Edge ((TKey, TKey) pair, float weight, EdgeType type = EdgeType.Null) {
		if (pair.Item1 == null || pair.Item2 == null) 
			throw new ArgumentNullException(nameof(pair));
		Vertices = new KeyPair<TKey>(pair);
		Weight = weight;
		Type = type;
	}
	public TKey Other([NotNull] TKey key) => Vertices.Other(key);

	public override string ToString () => Tuple + ( Mathf.Abs(Weight - 1f) < Mathf.Epsilon ? "" : " [ " + Weight + " ]") +
	                                      (Type != EdgeType.Null ? ": " + Type : "");
}
}