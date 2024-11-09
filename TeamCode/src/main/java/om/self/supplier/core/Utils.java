package om.self.supplier.core;

/**
 * Stores useful methods for the code. These can also be used outside this library
 */
public class Utils {

    public static<T> T convertNumber(Number number, T ref){
        return (T)convertNumber(number, ref.getClass());
    }

    /**
     * Tries to convert the input number to the 6 primitive number types else just tries to type cast.
     * @param number the number to be converted
     * @param ref the class of the type you want the number to be converted to
     * @return the converted number
     * @param <T> the type to convert to
     */
    public static<T> T convertNumber(Number number, Class<T> ref){
        if(ref == Integer.class) return (T)(Number)number.intValue();
        else if (ref == Double.class) return (T)(Number)number.doubleValue();
        else if (ref == Float.class) return (T)(Number)number.floatValue();
        else if (ref == Short.class) return (T)(Number)number.shortValue();
        else if (ref == Long.class) return (T)(Number)number.longValue();
        else if (ref == Byte.class) return (T)(Number)number.byteValue();
        else return (T)number;
    }
}
