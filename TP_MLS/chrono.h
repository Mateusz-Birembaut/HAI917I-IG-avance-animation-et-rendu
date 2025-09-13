#ifndef CHRONO_H
#define CHRONO_H

#include <chrono>
#include <iostream>

class Chrono{



	public:
		void start(){
			m_startTime = std::chrono::steady_clock::now();
		}

		void end(){
			const auto end = std::chrono::steady_clock::now();
			std::cout << "Temps écoulé : " << std::chrono::duration_cast<std::chrono::seconds>(end - m_startTime).count() << " s\n";
		}

	private:
		std::chrono::time_point<std::chrono::steady_clock> m_startTime;

};

#endif // CHRONO_H