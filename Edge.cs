using System;
using JetBrains.Annotations;
using UnityEngine;

namespace Utils.ProcGenUtils.GraphModel {
public class Edge<TKey> {
	[NotNull] internal readonly KeyPair<TKey> Vertices;
	public (TKey, TKey) Tuple => Vertices.Tuple;
	public (TKey, TKey) TransposedTuple => Vertices.TransposedTuple;
	public float Weight;

	public Edge(Edge<TKey> other) : this(other.Tuple, other.Weight) { }
	public Edge ([NotNull] TKey vKey, [NotNull] TKey uKey, float weight) : this((vKey, uKey), weight) { }
	public Edge ((TKey, TKey) pair, float weight) {
		if (pair.Item1 == null || pair.Item2 == null) 
			throw new ArgumentNullException(nameof(pair));
		Vertices = new KeyPair<TKey>(pair);
		Weight = weight;
	}
	public TKey Other([NotNull] TKey key) => Vertices.Other(key);

	public override string ToString () => Tuple + ( Mathf.Abs(Weight - 1f) < Mathf.Epsilon ? "" : " [ " + Weight + " ]");
}
}