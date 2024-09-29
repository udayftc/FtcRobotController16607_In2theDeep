
package TrcCommonLib.trclib;

/**
 * This interface implements the method to read the value from a value source.
 */
public interface TrcValueSource<T>
{
    /**
     * This method reads the value from the value source.
    *
    * @return value.
    */
    T getValue();

}   //interface TrcValueSource
