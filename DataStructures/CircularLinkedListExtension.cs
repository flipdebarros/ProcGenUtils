using System.Collections.Generic;

namespace Utils.ProcGenUtils.DataStructures {

public static class CircularLinkedListExtension {
	public static LinkedListNode<T> NextCircular<T> (this LinkedListNode<T> node) => node.Next ?? node.List.First;
	public static LinkedListNode<T> PreviousCircular<T> (this LinkedListNode<T> node) => node.Previous ?? node.List.Last;
}

}