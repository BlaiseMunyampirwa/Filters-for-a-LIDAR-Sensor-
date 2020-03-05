/* Testing Range and Temporal filter for LIDAR generated scans
 * Blaise Munyampirwa
 * University of Chicago. March, 2020
 */

#include <cstdlib>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <algorithm>
using namespace std;


class filter {
private:
	float minimum = 0.03;
	float maximum = 50.0;

public:
	filter() {}
	//range filter
	vector<float> update_range(vector<float> &scan);
	//temporal median filter
	vector<float> update_median(vector<vector<float> > &data);
	vector<float> generate_scans(unsigned int num);
	void add_scan(vector<float> &scan,
				  vector<vector<float> > &data);

	//find the median in the array.
	float find_median(vector<float> &input);
	~filter() {}
};

//Generate a scan based on the minimum and maximum value
vector<float> filter::update_range(vector<float> &scan) {
	float minimum = 0.03;
	float maximum = 50.;
	vector<float> output(scan.size(), minimum);
	for (unsigned int i = 0; i < scan.size(); i++) {
		if (scan[i] < minimum) {
			output[i] = minimum;
		}else if (scan[i] > maximum) {
			output[i] = maximum;
		} else {
			output[i] = scan[i];
		}
	}
	return output;
}

//Calculate the median for a sorted array
float filter::find_median(vector<float> & input) {
	unsigned int num = input.size();
	if (num % 2 == 0) {
			return 0.5 * (input[num / 2 - 1] + input[num / 2]);
	} else {
		return input[num / 2];
	}
}

//Generate a filtered scan based on the previous and the current measurements.
vector<float> filter::update_median(vector<vector<float> > & data) {
	unsigned int num = data.size();
	vector<float> med_out(num, 0);
	for (unsigned int i = 0; i < num; i++) {
		med_out[i] = find_median(data[i]);
	}
	return med_out;
}

//Generate random scans for testing
vector<float> filter::generate_scans(unsigned int num) {
	vector<float> input(num, 0);
	for (unsigned int i = 0; i < num; i++) {
		input[i] = rand() % 10;
	}
	return input;
}

//Add current scan to the previous ones
void filter::add_scan(vector<float> & scan, vector<vector<float> > & data) {
	unsigned int num = scan.size();
	unsigned int size = data.size();
	if (size == 0) {
		for (unsigned int i = 0; i < num; i++) {
			data.push_back(vector<float>());
			data[i].push_back(scan[i]);
		}
	} else {
		for (unsigned int i = 0; i < num; i++) {
			vector<float>::iterator it = lower_bound(data[i].begin(), data[i].end(), scan[i]);
			data[i].insert(it, scan[i]);
		}
	}
	return;
}