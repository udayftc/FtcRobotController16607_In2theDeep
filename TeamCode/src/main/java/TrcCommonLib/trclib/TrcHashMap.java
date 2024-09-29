
package TrcCommonLib.trclib;

import java.util.HashMap;
import java.util.NoSuchElementException;

/**
 * This class implements the TrcHashMap. TrcHashMap extends HashMap and added the add method so you can easily
 * append multiple hash map entries in a single statement. It can also differentiate the case of key not found
 * from the case of key maps to null. In the case of key not found, instead of returning null, it throws an
 * exception. TrcHashMap can accommodate different types of object in the same map. So it provides methods to
 * get the appropriate data type from the map. It throws an exception if the type is not what you expect.
 */
public class TrcHashMap<K, V> extends HashMap<K, V>
{
    /**
     * This method adds an entry to the hash map and returns this hashmap object so
     * you can chain multiple calls to the add method in a single statement.
     *
     * @param key   specifies the key of the mapping.
     * @param value specifies the value of the mappng.
     * @return this hashmap instance.
     */
    public TrcHashMap<K, V> add(K key, V value)
    {
        put(key, value);
        return this;
    }   //add

    /**
     * This method returns the value mapped the given key and throws an exception if the key cannot be found in the
     * hashmap.
     *
     * @param key specifies the key of the mapping to look for.
     * @return the value mapped to the key.
     * @throws NoSuchElementException if key is not in hashmap.
     */
    @Override
    public V get(Object key)
    {
        if (!containsKey(key))
        {
            throw new NoSuchElementException("\"" + key + "\"" + " not found.");
        }

        return super.get(key);
    }   //get

    /**
     * This method returns the integer value mapped the given key and throws an exception if the key cannot be found
     * in the hashmap or if the value is not the correct type.
     *
     * @param key specifies the key of the mapping to look for.
     * @return the integer value mapped to the key.
     * @throws IllegalArgumentException if key is not in hashmap.
     * @throws ClassCastException if value is not the correct type.
     */
    public Integer getInteger(Object key)
    {
        return (Integer)get(key);
    }   //getInteger

    /**
     * This method returns the float value mapped the given key and throws an exception if the key cannot be found
     * in the hashmap or if the value is not the correct type.
     *
     * @param key specifies the key of the mapping to look for.
     * @return the float value mapped to the key.
     * @throws IllegalArgumentException if key is not in hashmap.
     * @throws ClassCastException if value is not the correct type.
     */
    public Float getFloat(Object key)
    {
        return (Float)get(key);
    }   //getFloat

    /**
     * This method returns the double value mapped the given key and throws an exception if the key cannot be found
     * in the hashmap or if the value is not the correct type.
     *
     * @param key specifies the key of the mapping to look for.
     * @return the double value mapped to the key.
     * @throws IllegalArgumentException if key is not in hashmap.
     * @throws ClassCastException if value is not the correct type.
     */
    public Double getDouble(Object key)
    {
        return (Double)get(key);
    }   //getDouble

    /**
     * This method returns the boolean value mapped the given key and throws an exception if the key cannot be found
     * in the hashmap or if the value is not the correct type.
     *
     * @param key specifies the key of the mapping to look for.
     * @return the boolean value mapped to the key.
     * @throws IllegalArgumentException if key is not in hashmap.
     * @throws ClassCastException if value is not the correct type.
     */
    public Boolean getBoolean(Object key)
    {
        return (Boolean)get(key);
    }   //getBoolean

    /**
     * This method returns the string value mapped the given key and throws an exception if the key cannot be found
     * in the hashmap or if the value is not the correct type.
     *
     * @param key specifies the key of the mapping to look for.
     * @return the string value mapped to the key.
     * @throws IllegalArgumentException if key is not in hashmap.
     * @throws ClassCastException if value is not the correct type.
     */
    public String getString(Object key)
    {
        return (String)get(key);
    }   //getString

}   //class TrcHashMap
