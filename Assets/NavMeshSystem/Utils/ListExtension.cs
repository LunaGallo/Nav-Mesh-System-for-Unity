using System;
using System.Collections.Generic;

public static partial class ListExtension {

    public static T RandomElement<T>(this List<T> list) {
        return list[UnityEngine.Random.Range(0, list.Count)];
    }

    public static T FindThruIndex<T>(this List<T> list, Predicate<int> indexPredicate) {
        for(int i=0; i<list.Count; i++) {
            if (indexPredicate(i)) {
                return list[i];
            }
        }
        return default(T);
    }
    public static List<T> FindAllThruIndex<T>(this List<T> list, Predicate<int> indexPredicate) {
        List<T> result = new List<T>();
        for (int i = 0; i < list.Count; i++) {
            if (indexPredicate(i)) {
                result.Add(list[i]);
            }
        }
        return result;
    }

    public static bool FindMinimum<T>(this List<T> list, Func<T, float> func, out float minValue, out int minIndex) {
        minValue = float.MaxValue;
        minIndex = -1;
        for(int i=0; i<list.Count; i++) {
            float curValue = func(list[i]);
            if (minValue > curValue) {
                minValue = curValue;
                minIndex = i;
            }
        }
        return (minIndex > -1);
    }
    public static bool FindMinimum<T>(this List<T> list, Func<T, float> func, out float minValue) {
        minValue = float.MaxValue;
        int minIndex = -1;
        for (int i = 0; i < list.Count; i++) {
            float curValue = func(list[i]);
            if (minValue > curValue) {
                minValue = curValue;
                minIndex = i;
            }
        }
        return (minIndex > -1);
    }
    public static bool FindMinimum<T>(this List<T> list, Func<T, float> func, out int minIndex) {
        float minValue = float.MaxValue;
        minIndex = -1;
        for (int i = 0; i < list.Count; i++) {
            float curValue = func(list[i]);
            if (minValue > curValue) {
                minValue = curValue;
                minIndex = i;
            }
        }
        return (minIndex > -1);
    }

    public static bool FindMaximum<T>(this List<T> list, Func<T, float> func, out float maxValue, out int maxIndex) {
        maxValue = float.MinValue;
        maxIndex = -1;
        for (int i = 0; i < list.Count; i++) {
            float curValue = func(list[i]);
            if (maxValue < curValue) {
                maxValue = curValue;
                maxIndex = i;
            }
        }
        return (maxIndex > -1);
    }
    public static bool FindMaximum<T>(this List<T> list, Func<T, float> func, out float maxValue) {
        maxValue = float.MinValue;
        int maxIndex = -1;
        for (int i = 0; i < list.Count; i++) {
            float curValue = func(list[i]);
            if (maxValue < curValue) {
                maxValue = curValue;
                maxIndex = i;
            }
        }
        return (maxIndex > -1);
    }
    public static bool FindMaximum<T>(this List<T> list, Func<T, float> func, out int maxIndex) {
        float maxValue = float.MinValue;
        maxIndex = -1;
        for (int i = 0; i < list.Count; i++) {
            float curValue = func(list[i]);
            if (maxValue < curValue) {
                maxValue = curValue;
                maxIndex = i;
            }
        }
        return (maxIndex > -1);
    }

    public static T SumValue<T>(this List<T> list, Func<T,T,T> addFunc) {
        T result = default(T);
        list.ForEach(e => result = addFunc(result, e));
        return result;
    }
    public static V SumElement<V,E>(this List<E> list, Func<V, E, V> addFunc) {
        V result = default(V);
        list.ForEach(e => result = addFunc(result, e));
        return result;
    }
    public static V SumElementPairs<V, E>(this List<E> list, Func<V, E, E, V> addFunc, bool ciclic = false, bool jumpUsedElement = false) {
        V result = default(V);
        list.ForEachConsecutivePair((f,s) => result = addFunc(result, f,s), ciclic, jumpUsedElement);
        return result;
    }

    public static T AverageValue<T>(this List<T> list, Func<T,T,T> addFunc, Func<T,int,T> divFunc) {
        return divFunc(list.SumValue(addFunc), list.Count);
    }
    public static V AverageElement<V, E>(this List<E> list, Func<V, E, V> addFunc, Func<V, int, V> divFunc) {
        return divFunc(list.SumElement(addFunc), list.Count);
    }

    public static void AddIfDoesntContain<T>(this List<T> list, T element) {
        if (!list.Contains(element)) {
            list.Add(element);
        }
    }
    public static void AddRangeIfDoesntContain<T>(this List<T> list, IEnumerable<T> elements) {
        foreach(T element in elements) {
            if (!list.Contains(element)) {
                list.Add(element);
            }
        }
    }

    public static void ForEachConsecutivePair<T>(this List<T> list, Action<T,T> action, bool ciclic = false, bool jumpUsedElement = false) {
        if (list.Count < 2) {
            return;
        }
        int increment = (jumpUsedElement)? 2:1;
        int size = (ciclic)? list.Count:list.Count-1;
        for (int i=0; i<size; i+=increment) {
            int j = (i+1)%list.Count;
            action.Invoke(list[i], list[j]);
        }
    }
    public static void ForEachConsecutiveTrio<T>(this List<T> list, Action<T, T, T> action, bool ciclic = false, bool jumpUsedElements = false) {
        if (list.Count < 3) {
            return;
        }
        int increment = (jumpUsedElements) ? 3 : 1;
        int size = (ciclic) ? list.Count : list.Count - 1;
        for (int i = 0; i < size; i += increment) {
            int j = (i + 1) % list.Count;
            int k = (j + 1) % list.Count;
            action.Invoke(list[i], list[j], list[k]);
        }
    }

}
