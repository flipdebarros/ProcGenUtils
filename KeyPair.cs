using System;
using System.Collections.Generic;
using JetBrains.Annotations;

namespace Utils.ProcGenUtils.GraphModel {

internal class KeyPair<TKey> {
	[NotNull] public readonly TKey Item1;
	[NotNull] public readonly TKey Item2;
	public (TKey, TKey) Tuple => (Item1, Item2);
	public (TKey, TKey) TransposedTuple => (Item2, Item1);

	public TKey this[int i] => i switch { 0 => Item1, 1 => Item2, _ => throw new IndexOutOfRangeException() };  
	public KeyPair([NotNull] TKey item1, [NotNull] TKey item2) {
		Item1 = item1 ?? throw new ArgumentNullException(nameof(item1));
		Item2 = item2 ?? throw new ArgumentNullException(nameof(item2));
	}
	public KeyPair([NotNull] KeyPair<TKey> pair) {
		if (pair == null) throw new ArgumentNullException(nameof(pair));
		Item1 = pair[0] ?? throw new ArgumentNullException(nameof(Item1));
		Item2 = pair[1] ?? throw new ArgumentNullException(nameof(Item2));
	}
	public KeyPair([NotNull] (TKey, TKey) pair) : this(pair.Item1, pair.Item2) { }
	public TKey Other([NotNull] TKey key) {
		if (key == null) throw new ArgumentNullException(nameof(key));
		return key.Equals(Item1) ? Item2 : key.Equals(Item2) ? Item1 : throw new ArgumentException("Invalid key.");
	}
	public KeyPair<TKey> Transpose => new KeyPair<TKey>(Item2, Item1);

	public void Deconstruct ([NotNull] out TKey item1, [NotNull] out TKey item2) {
		item1 = Item1;
		item2 = Item2;
	}

	public override string ToString () => "( " + Item1 + ", " + Item2 + ")";

	#region Comparators

	private sealed class UndirectedEqualityComparer : IEqualityComparer<KeyPair<TKey>> {
		public bool Equals(KeyPair<TKey> x, KeyPair<TKey> y) {
			if (ReferenceEquals(x, y)) return true;
			if (ReferenceEquals(x, null)) return false;
			if (ReferenceEquals(y, null)) return false;
			if (x.GetType() != y.GetType()) return false;
			return EqualityComparer<TKey>.Default.Equals(x.Item1, y.Item1) && EqualityComparer<TKey>.Default.Equals(x.Item2, y.Item2) ||
			       EqualityComparer<TKey>.Default.Equals(x.Item1, y.Item2) && EqualityComparer<TKey>.Default.Equals(x.Item2, y.Item1);
		}
		public int GetHashCode(KeyPair<TKey> obj) {
			return EqualityComparer<TKey>.Default.GetHashCode(obj.Item1) | EqualityComparer<TKey>.Default.GetHashCode(obj.Item2);
		}
	}
	public static IEqualityComparer<KeyPair<TKey>> UndirectedComparer { get; } = new UndirectedEqualityComparer();
	
	private sealed class DirectedEqualityComparer : IEqualityComparer<KeyPair<TKey>> {
		public bool Equals(KeyPair<TKey> x, KeyPair<TKey> y) {
			if (ReferenceEquals(x, y)) return true;
			if (ReferenceEquals(x, null)) return false;
			if (ReferenceEquals(y, null)) return false;
			if (x.GetType() != y.GetType()) return false;
			return EqualityComparer<TKey>.Default.Equals(x.Item1, y.Item1) && EqualityComparer<TKey>.Default.Equals(x.Item2, y.Item2);
		}
		public int GetHashCode(KeyPair<TKey> obj) {
			unchecked {
				return (EqualityComparer<TKey>.Default.GetHashCode(obj.Item1) * 397) ^ EqualityComparer<TKey>.Default.GetHashCode(obj.Item2);
			}
		}
	}
	public static IEqualityComparer<KeyPair<TKey>> DirectedComparer { get; } = new DirectedEqualityComparer();
	
	#endregion
}

}