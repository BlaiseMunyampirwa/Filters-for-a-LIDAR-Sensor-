/* Testing Range and Temporal filter for LIDAR generated scans
 * Blaise Munyampirwa
 * University of Chicago. March, 2020
 */


#include "filters.h"
#include "testcases.h"

int main(){
	// run testcases
	Test CHECK;
	CHECK.tests();

	// Range filter
    filter filter1; 
    vector<float> scan1 = {0., 4., 7., 9.}; 
    vector<float> output1 = filter1.update_range(scan1); 

    // Temporal median filter
	filter filter2;
    vector<float> scan2 = {10., 4., -7., 3.};
    vector<vector<float> > data; // Collected data up to current scan
	filter2.add_scan(scan2, data); // Add new scan to the data
	vector<float> output2 = filter2.update_median(data); 
	return 0;
}