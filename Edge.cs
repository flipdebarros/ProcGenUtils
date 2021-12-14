using System;
using JetBrains.Annotations;

namespace Utils.ProcGenUtils.GraphModel {
public class Edge<TKey> {
	[NotNull] public readonly KeyPair<TKey> Vertices;
	public TKey VKey => Vertices.Item1;
	public TKey UKey => Vertices.Item2;
	public float Weight;

	public Edge(Edge<TKey> other) : this(new KeyPair<TKey>(other.Vertices), other.Weight) { }
	public Edge ([NotNull] TKey vKey, [NotNull] TKey uKey, float weight) : 
		this(new KeyPair<TKey>(vKey, uKey), weight) { }
	public Edge ([NotNull] KeyPair<TKey> pair, float weight) {
		if (pair == null || pair[0] == null || pair[1] == null) 
			throw new ArgumentNullException(nameof(pair));
		Vertices = pair;
		Weight = weight;
	}
	public TKey Other([NotNull] TKey key) => Vertices.Other(key);

	public override string ToString () => "[ " + VKey + ", " + UKey + " ] ( " + Weight + " )";
}
}