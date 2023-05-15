#pragma once

// CPUData - container of stats for a CPU

#include <string>

class CPUData
{
public:
    /**
     * @brief A method that reads data from a given string and updates the statistics of the CPU.
     * @param line Represents a line of data for a particular CPU
     */
	void ReadData(const std::string & line);
	/**
     	 * @brief A method that computes the total active time of the CPU.
	 * @return Total active time of CPU
	 * @see #CPUStates for definitions for all types of states of CPU
	 * @note #S_USER, #S_NICE, #S_SYSTEM, #S_IRQ, #S_SOFTIRQ, #S_STEAL, #S_GUEST, #S_GUEST_NICE are the active states.
     */
	std::size_t GetActiveTime() const;
	 /**
     	 * @brief A function that calculates the total idle time of the CPU.
	 * @return Total idle time of the CPU
	 * @see #S_IDLE describes what idle state is
	 * @note #S_IDLE, #S_IOWAIT are the idle states.
     	*/
    	std::size_t GetIdleTime() const;
	/**
	* @brief A method which computes the time spent in a specific state of the CPU.
     	* @param state state of the CPU
     	* @return Time spent by a CPU in a specific state.
	* @see #CPUStates for definitions for all types of states of CPU
    	*/
	std::size_t GetStateTime(unsigned int state) const;
	 /**
   	 * @brief A function that calculates the total time spent by the CPU on all states.
	* @return Total time spent by the CPU.
    	*/
	std::size_t GetTotalTime() const;
	 /**
    	* @return A function that calculates the label of the CPU.
    	* @see #CPUStates for definitions for all types of states of CPU
	*/
	const std::string & GetLabel() const;

public:
	/**
	* @brief determines if the input string contains CPU statistics.
	* @note CPU statistics means the data related to the usage and state of a CPU instance in a system.
	*/
	static bool IsDataCPUStats(const std::string & line);

public:
	enum CPUStates
	{
	/// The time spent by normal processes executing in user mode.
		S_USER = 0,
	/// The time spent by lower priority processes executing in user mode.
	    S_NICE,
	/// Time spent by processes executing in kernel mode.
	    S_SYSTEM,
	 /// Time waiting for a task to be given or time spent doing nothing.
	    S_IDLE,
	/// Time spent waiting for I/O to complete
	    S_IOWAIT,
	/// Time spent servicing interrupts.
	    S_IRQ,
	/// Time spent servicing softirqs.
	    S_SOFTIRQ,
	/// Time spent in other operating systems.
	    S_STEAL,
	  /// Time spent running a virtual CPU for a guest operating system.
	    S_GUEST,
	/// Time spent running a lower priority guest CPU for a guest operating system.
	    S_GUEST_NICE,
	/// The total number of CPU states.	
		NUM_CPU_STATES
	};

private:
	static const std::string STR_CPU;
	static const std::string STR_TOT;

	static const std::size_t LEN_STR_CPU;

private:
	std::string mLabel;

	std::size_t mTimes[NUM_CPU_STATES];
};

// == INLINE FUNCTIONS ==

inline 	std::size_t CPUData::GetActiveTime() const
{
	return	mTimes[S_USER] +
			mTimes[S_NICE] +
			mTimes[S_SYSTEM] +
			mTimes[S_IRQ] +
			mTimes[S_SOFTIRQ] +
			mTimes[S_STEAL] +
			mTimes[S_GUEST] +
			mTimes[S_GUEST_NICE];
}

inline std::size_t CPUData::GetIdleTime() const
{
	return mTimes[S_IDLE] + mTimes[S_IOWAIT];
}

inline std::size_t CPUData::GetStateTime(unsigned int state) const
{
	if(state < NUM_CPU_STATES)
		return mTimes[state];
	else
		return 0;
}

inline std::size_t CPUData::GetTotalTime() const
{
	return	mTimes[S_USER] +
			mTimes[S_NICE] +
			mTimes[S_SYSTEM] +
			mTimes[S_IDLE] +
			mTimes[S_IOWAIT] +
			mTimes[S_IRQ] +
			mTimes[S_SOFTIRQ] +
			mTimes[S_STEAL] +
			mTimes[S_GUEST] +
			mTimes[S_GUEST_NICE];
}

inline const std::string & CPUData::GetLabel() const { return mLabel; }

inline bool CPUData::IsDataCPUStats(const std::string & line)
{
	return (!line.compare(0, LEN_STR_CPU, STR_CPU));
}
