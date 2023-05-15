#pragma once

// CPUSnapshot - a snapshot of all the CPU data from /proc/stat at a given time

#include "CPUData.h"

/// @brief Provides a snapshot of all the CPU data from /proc/stat at a given time

#include <vector>

class CPUSnapshot
{
public:
	/// Constructor
	CPUSnapshot();
	/// @brief Getter method for the number of entries in CPU time data
	/// @return Number of entries
	std::size_t GetNumEntries() const;
	
	/// @brief Finds the label of the total CPU time
	/// @return Pointer to the label of the total CPU time
	const char * GetLabelTotal() const;
	
	/// @brief Finds the label of a specific CPU identitfied by its CPU number.
	/// @param cpu CPU Number
	/// @return Pointer to the C-style string of the label of a specific CPU
	const char * GetLabel(unsigned int cpu) const;

	/// @brief Finds the total Active Time the entries of the CPU Data
	/// @return Total Active Time
	std::size_t GetActiveTimeTotal() const;
	
	/// @brief Computes the active CPU Time for a specifc CPU identitfied by its CPU number.
	/// @param cpu CPU Number or CPU identifier
	/// @return Active CPU Time
	std::size_t GetActiveTime(unsigned int cpu) const;

	/// @brief Calculates the total idle CPU Time for all CPU's
	/// @return total idle CPU Time
	std::size_t GetIdleTimeTotal() const;
	
	/// @brief Computes the idle time for a specific CPU identitfied by its CPU number.
	/// @param cpu CPU Number or CPU identifier
	/// @return Specific idle time
	std::size_t GetIdleTime(unsigned int cpu) const;

	/// @brief Finds the total time spent in a specific CPU state
	/// @param state state of the CPU
	/// @return Total spent time in a state
	std::size_t GetStateTimeTotal(unsigned int state) const;
	
	/// @brief Calculates the time spent on a specific state for a specific CPU 
	/// @param state CPU'S State
	/// @param cpu CPU identifier or CPU Number
	/// @return Time spent on state for a CPU	
	std::size_t GetStateTime(unsigned int state, unsigned int cpu) const;
	
	/// @brief Computes the total CPU Time for all CPU's
	/// @return Total CPU Time
	std::size_t GetTotalTimeTotal() const;
	
	/// @brief Finds the CPU time for a specific CPU
	/// @param cpu CPU identifier or CPU Number
	/// @return CPU Time for a CPU
	std::size_t GetTotalTime(unsigned int cpu) const;

private:
	static const int INDEX_TOT;

private:
	std::vector<CPUData> mEntries;
};

// == INLINE FUNCTIONS ==

inline std::size_t CPUSnapshot::GetNumEntries() const { return mEntries.size() - 1; }

inline const char * CPUSnapshot::GetLabelTotal() const { return mEntries[INDEX_TOT].GetLabel().c_str(); }

inline const char * CPUSnapshot::GetLabel(unsigned int cpu) const
{
	// skip total
	++cpu;

	if(cpu < mEntries.size())
		return mEntries[cpu].GetLabel().c_str();
	else
		return nullptr;
}

inline std::size_t CPUSnapshot::GetActiveTimeTotal() const { return mEntries[INDEX_TOT].GetActiveTime(); }

inline std::size_t CPUSnapshot::GetActiveTime(unsigned int cpu) const
{
	// skip total
	++cpu;

	if(cpu < mEntries.size())
		return mEntries[cpu].GetActiveTime();
	else
		return 0;
}

inline std::size_t CPUSnapshot::GetIdleTimeTotal() const { return mEntries[INDEX_TOT].GetIdleTime(); }

inline std::size_t CPUSnapshot::GetIdleTime(unsigned int cpu) const
{
	// skip total
	++cpu;

	if(cpu < mEntries.size())
		return mEntries[cpu].GetIdleTime();
	else
		return 0;
}

inline std::size_t CPUSnapshot::GetStateTimeTotal(unsigned int state) const { return mEntries[INDEX_TOT].GetStateTime(state); }

inline std::size_t CPUSnapshot::GetStateTime(unsigned int state, unsigned int cpu) const
{
	// skip total
	++cpu;

	if(cpu < mEntries.size())
		return mEntries[cpu].GetStateTime(state);
	else
		return 0;
}

inline std::size_t CPUSnapshot::GetTotalTimeTotal() const { return mEntries[INDEX_TOT].GetTotalTime(); }

inline std::size_t CPUSnapshot::GetTotalTime(unsigned int cpu) const
{
	// skip total
	++cpu;

	if(cpu < mEntries.size())
		return mEntries[cpu].GetTotalTime();
	else
		return 0;
}
