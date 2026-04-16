package lab9;

public class Semaphore {

    private int value; // stores the number of available resources 
    
    // value tracks how many threads can enter
    // value = 0 no resources --> must wait 
    // value = 1 one resource available 

    public Semaphore(int value) {
        this.value = value; // setting the initial value in the constructor
    }

    public synchronized void up() { // the release / signal operation
        value++; // increases available resource
        notify(); // wakes up ONE waiting thread
    }

    public synchronized void down() { // the acquire / wait operation
        while (value == 0) { // if no resources, thread must wait
            try {
                wait(); // puts the thread to sleep / blocked
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
        }
        value--; // thread enters critical section, decrement
    }
}