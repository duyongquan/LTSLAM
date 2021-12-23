#ifndef XSLAM_TUTORIAL_RANDOM_H
#define XSLAM_TUTORIAL_RANDOM_H

#include <cstdlib>
#include <vector>

namespace xslam {
namespace opencv {

// Functions to generate pseudo-random numbers
class Random
{
public:
    class UnrepeatedRandomizer;
  
public:
	// Sets the random number seed to the current time
	static void SeedRand();
	
	// Sets the random number seed to the current time only the first
	// time this function is called
	static void SeedRandOnce();

	// Sets the given random number seed
	static void SeedRand(int seed);

	// Sets the given random number seed only the first time this function 
	// is called
	static void SeedRandOnce(int seed);

	// Returns a random number in the range [0..1]
	template <class T>
	static T RandomValue()
    {
		return (T)rand()/(T)RAND_MAX;
	}

	// Returns a random number in the range [min..max]
	template <class T>
	static T RandomValue(T min, T max)
    {
		return Random::RandomValue<T>() * (max - min) + min;
	}

    // Returns a random int in the range [min..max]
	static int RandomInt(int min, int max);
	
	// Returns a random number from a gaussian distribution
	template <class T>
	static T RandomGaussianValue(T mean, T sigma)
	{
        // Box-Muller transformation
        T x1, x2, w, y1;

        do {
            x1 = (T)2. * RandomValue<T>() - (T)1.;
            x2 = (T)2. * RandomValue<T>() - (T)1.;
            w = x1 * x1 + x2 * x2;
        } while ( w >= (T)1. || w == (T)0. );

        w = sqrt( ((T)-2.0 * log( w ) ) / w );
        y1 = x1 * w;

        return( mean + y1 * sigma );
	}

private:
    // If SeedRandOnce() or SeedRandOnce(int) have already been called
    static bool m_already_seeded;
};


/// Provides pseudo-random numbers with no repetitions
class Random::UnrepeatedRandomizer
{
public:
    UnrepeatedRandomizer(int min, int max);
    ~UnrepeatedRandomizer(){}
    UnrepeatedRandomizer(const UnrepeatedRandomizer& rnd);
    UnrepeatedRandomizer& operator=(const UnrepeatedRandomizer& rnd);
    int get();
    inline bool empty() const { return m_values.empty(); }
    inline unsigned int left() const { return m_values.size(); }
    void reset();
    void createValues();

protected:
    // Min of range of values
    int m_min;
    // Max of range of values
    int m_max;

    // Available values
    std::vector<int> m_values;
};

} // namespace opencv
} // namespace xslam

#endif // XSLAM_TUTORIAL_RANDOM_H