package lab9;

/**
 * COE 628 - Lab 9: Dining Philosophers Problem
 *
 * Part A: Java translation of the C algorithm (Figure 6.13 - Second Solution)
 *         Uses a 'room' semaphore initialized to 4 to prevent deadlock.
 *         At most 4 of the 5 philosophers can attempt to pick up forks at once,
 *         breaking the circular wait condition.
 *
 * Part B: Uses custom Semaphore class from Lab 8 (up/down operations).
 * Part C: Produces sample output matching the Appendix.
 */
public class DiningPhilosophers {

    // 5 fork semaphores, each initialized to 1 (binary semaphore / mutex)
    static Semaphore[] fork = new Semaphore[5];

    // Room semaphore initialized to 4 — only 4 philosophers allowed in "dining room" at once
    // This prevents circular wait / deadlock (Figure 6.13 second solution)
    static Semaphore room = new Semaphore(4);

    // Track how many philosophers have completed dinner
    static int completedCount = 0;

    // Track which philosophers have completed (for the status print)
    static boolean[] completed = new boolean[5];

    static {
        for (int i = 0; i < 5; i++) {
            fork[i] = new Semaphore(1);
        }
    }

    static class Philosopher extends Thread {
        int id;

        Philosopher(int id) {
            this.id = id;
        }

        void think() {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        void eat() {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        @Override
        public void run() {
            think();

            // Enter room — at most 4 philosophers inside at once (prevents deadlock)
            room.down();

            // Pick up left fork (fork[i])
            synchronized (DiningPhilosophers.class) {
                System.out.println("Fork " + (id + 1) + " taken by Philosopher " + (id + 1));
            }
            fork[id].down();

            // Announce waiting for right fork
            int rightFork = (id + 1) % 5;
            synchronized (DiningPhilosophers.class) {
                System.out.println("Philosopher " + (id + 1) + " is waiting for Fork " + (rightFork + 1));
                printStatus();
            }

            // Pick up right fork (fork[(i+1) mod 5])
            fork[rightFork].down();

            synchronized (DiningPhilosophers.class) {
                System.out.println("Fork " + (rightFork + 1) + " taken by Philosopher " + (id + 1));
            }

            // Eat
            eat();

            // Done — release forks and leave room
            synchronized (DiningPhilosophers.class) {
                completedCount++;
                completed[id] = true;
                System.out.println("Philosopher " + (id + 1) + " completed his dinner");
                System.out.println("Philosopher " + (id + 1) + " released fork " + (id + 1) + " and fork " + (rightFork + 1));
            }

            fork[rightFork].up();  // signal(fork[(i+1) mod 5])
            fork[id].up();         // signal(fork[i])
            room.up();             // signal(room)

            synchronized (DiningPhilosophers.class) {
                printStatus();
            }
        }
    }

    static synchronized void printStatus() {
        for (int i = 0; i < 5; i++) {
            if (completed[i]) {
                System.out.println("Philosopher " + (i + 1) + " completed his dinner");
            }
        }
        System.out.println("Till now num of philosophers completed dinner are " + completedCount);
    }

    public static void main(String[] args) throws InterruptedException {
        // parbegin(philosopher(0), philosopher(1), ..., philosopher(4))
        Philosopher[] philosophers = new Philosopher[5];
        for (int i = 0; i < 5; i++) {
            philosophers[i] = new Philosopher(i);
        }
        for (int i = 0; i < 5; i++) {
            philosophers[i].start();
        }
        for (int i = 0; i < 5; i++) {
            philosophers[i].join();
        }
    }
}