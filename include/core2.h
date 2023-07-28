//TaskHandle_t task_loop1;
//task2 task;

#include "task2.h"

	//private:

		exchange ob;
		boolean b;

		//
		// Locking code
		//
		SemaphoreHandle_t bigLock;


		// Lock helper functions
			// Hard coded to use "bigLock"
			void bigLock_lock() {
				while ((bigLock == NULL) || (xSemaphoreTake(bigLock, (TickType_t)0) == pdFALSE)) {
					delay(1);
				}
			}

			void bigLock_unlock() {
				xSemaphoreGive(bigLock);
			}


		// Protected resource that uses locking code
	//public:
		void set_ob(exchange nextExchange) {
			bigLock_lock();
			ob = nextExchange;
			bigLock_unlock();
		}

		// Use lock for read as well, not strictly necessary for a simple data strcuture
		// See Readers–writers problem
		exchange get_ob() {
			bigLock_lock();
			exchange x = ob;
			bigLock_unlock();
			return x;
		}
		//
		// End of locking code
		//

	exchange getNewExchange(exchange *oldExchange){
		exchange nextExchange = get_ob();
		set_ob(*oldExchange);
		return nextExchange;
	}

	exchange getNewExchange(exchange *oldExchange, boolean id){
		if(b == id){
			return *oldExchange;
		}
		exchange nextExchange = getNewExchange(oldExchange);
		b = id;
		return nextExchange;
	}


