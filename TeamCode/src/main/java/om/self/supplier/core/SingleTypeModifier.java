package om.self.supplier.core;

/**
 * Like modifier but only for things that have the same input and return type. This just helps give a more defined structure for linking modifiers.
 * @param <T> the input and return type
 * @see Modifier
 */
public interface SingleTypeModifier<T> extends Modifier<T,T> {
}
