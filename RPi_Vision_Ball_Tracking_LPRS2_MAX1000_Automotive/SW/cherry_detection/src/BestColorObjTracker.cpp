
#include "BestColorObjTracker.hpp"

#include "ColorObjDetector.hpp"
#include "PrintUtils.hpp"
#include "TrackballHelper.hpp"

#include <unordered_map>

#define MAX_SCREEN_W 1280
#define MAX_SCREEN_H 960

#define HISTORY_SIZE (25*2)
#define ESTABLISHMENT_TAKEOVER_PERCENTS 20

BestColorObjTracker::BestColorObjTracker(
	const string& track_cfg,
	const string& color_cfg,
	const string& img
) :
	cod(new ColorObjDetector(color_cfg, img))
{
	s.read(track_cfg);
	GET_DEFAULT_SETTING(s, middle_x, MAX_SCREEN_W/2);
	GET_DEFAULT_SETTING(s, middle_y, MAX_SCREEN_H/2);
	GET_DEFAULT_SETTING(s, size_th_0, 200);
	GET_DEFAULT_SETTING(s, size_th_1, 100);
	GET_DEFAULT_SETTING(s, dist_th_0, 200);
	GET_DEFAULT_SETTING(s, dist_th_1, 100);
	
#if 0
	TRACKBALL_WITH_CALLBACK(
		middle_x,
		MAX_SCREEN_W,
		[&](int value){
			middle.x = middle_x;
			UPDATE_TRACKBALL(middle_x);
			SET_AND_FLUSH_SETTING(s, middle_x);
			settings_changed();
		}
	);
	TRACKBALL_WITH_CALLBACK(
		middle_y,
		MAX_SCREEN_H,
		[&](int value){
			middle.y = middle_y;
			UPDATE_TRACKBALL(middle_y);
			SET_AND_FLUSH_SETTING(s, middle_y);
			settings_changed();
		}
	);
	
	TRACKBALL_WITH_CALLBACK(
		size_th_0,
		MAX_SCREEN_W,
		[&](int value){
			UPDATE_TRACKBALL(size_th_0);
			SET_AND_FLUSH_SETTING(s, size_th_0);
			settings_changed();
		}
	);
	TRACKBALL_WITH_CALLBACK(
		size_th_0,
		MAX_SCREEN_W,
		[&](int value){
			UPDATE_TRACKBALL(size_th_0);
			SET_AND_FLUSH_SETTING(s, size_th_0);
			settings_changed();
		}
	);
	TRACKBALL_WITH_CALLBACK(
		size_th_1,
		MAX_SCREEN_W,
		[&](int value){
			UPDATE_TRACKBALL(size_th_1);
			SET_AND_FLUSH_SETTING(s, size_th_1);
			settings_changed();
		}
	);
	TRACKBALL_WITH_CALLBACK(
		dist_th_0,
		MAX_SCREEN_W,
		[&](int value){
			UPDATE_TRACKBALL(dist_th_0);
			SET_AND_FLUSH_SETTING(s, dist_th_0);
			settings_changed();
		}
	);
	TRACKBALL_WITH_CALLBACK(
		dist_th_1,
		MAX_SCREEN_W,
		[&](int value){
			UPDATE_TRACKBALL(dist_th_1);
			SET_AND_FLUSH_SETTING(s, dist_th_1);
			settings_changed();
		}
	);
#endif
}

BestColorObjTracker::~BestColorObjTracker() {
	delete cod;
}

bool BestColorObjTracker::next_frame() {
	return cod->next_frame();
}
void BestColorObjTracker::settings_changed() {
	cod->settings_changed();
}

template <typename T>
vector<const T*> vec_2_ptr_vec(const vector<T>& in) {
	vector<const T*> out;
	for(auto& kp : in){
		out.push_back(&kp);
	}
	return out;
}

static vector<const KeyPoint*> sort_cloesest_to(
	const vector<KeyPoint>& in,
	Point2f to
) {
	vector<const KeyPoint*> out = vec_2_ptr_vec(in);
	sort(
		out.begin(),
		out.end(), 
		[&](const KeyPoint* a, const KeyPoint* b) -> bool {
			float dist_a = norm(a->pt - to);
			float dist_b = norm(b->pt - to);
			return dist_a < dist_b; 
		}
	);
	return out;
}
	
void BestColorObjTracker::track_best(
	vec3& cmd
) {
	cod->process_image();
	cod->blob_detect();
	//cod.mean_detect();
	
	//middle = Point(cod->src.cols/2, cod->src.rows/2);
	middle = Point(middle_x, middle_y);
	
	cod->draw_marker(middle, 0);
	
	Point2f m = middle;
	KeyPoint null_kp(m, 1000);
	if(best_history.size() == 0){
		best_history.push_front(null_kp);
	}
	const KeyPoint* best = &null_kp;
	
	if(!cod->blob_keypoints.empty()){
		
		const vector<KeyPoint>& kps = cod->blob_keypoints;
		
		int N = kps.size();
		
		vector<const KeyPoint*> closest_to_m_kps = sort_cloesest_to(kps, m);
		vector<const KeyPoint*> closest_to_prev_kps 
			= sort_cloesest_to(kps, best_history.front().pt);
		vector<const KeyPoint*> largest_kps = vec_2_ptr_vec(kps);
		sort(
			largest_kps.begin(),
			largest_kps.end(), 
			[](const KeyPoint* a, const KeyPoint* b) -> bool {
				return a->size > b->size; 
			}
		);
		const KeyPoint* closest = closest_to_m_kps[0];
		const KeyPoint* largest = largest_kps[0];
		
		// Filtering by voting
		unordered_map<const KeyPoint*, int> kp_to_points(10);
		for(auto& kp : kps){
			kp_to_points[&kp] = 0;
		}
		const int M = 3;
		for(int i = 0; i < min(N, M); i++){
			int points = M-i+1;
			kp_to_points[largest_kps[i]] += points;
			kp_to_points[closest_to_m_kps[i]] += points;
			kp_to_points[closest_to_prev_kps[i]] += points;
		}
		vector<const KeyPoint*> best_kps = vec_2_ptr_vec(kps);
		sort(
			best_kps.begin(),
			best_kps.end(), 
			[&](const KeyPoint* a, const KeyPoint* b) -> bool {
				return kp_to_points[a] > kp_to_points[b]; 
			}
		);
		best = best_kps[0];
		
		cod->draw_marker(closest_to_m_kps[0]->pt, 1);
		cod->draw_marker(closest_to_prev_kps[0]->pt, 2);
		cod->draw_marker(largest_kps[0]->pt, 3);
		cod->draw_marker(best_history.front().pt, 5);
	}
	
	best_history.push_front(*best);
	if(best_history.size() >= HISTORY_SIZE){
		best_history.pop_back();
	}
	
	cod->draw_marker(best->pt, 4);
	cod->show_results();
	
	Point2f d = best->pt - m;
	float dist = norm(d);
	float size = best->size;
	
	vec3 prop_cmd; // Proposal for cmd.
	auto yz_proc = [&](float dd) -> float{
		int s = dd > 0 ? -1 : +1;
		if(abs(dd) > dist_th_0){
			return s*1.0;
		}if(abs(dd) > dist_th_1){
			return s*0.5;
		}else{
			return 0.0;
		}
	};
	prop_cmd.y = yz_proc(d.x);
	prop_cmd.z = yz_proc(d.y);
	if(prop_cmd.y == 0 && prop_cmd.z == 0){
		if(size > size_th_0){
			prop_cmd.x = 0.0;
		}else if(size > size_th_1){
			prop_cmd.x = +0.5;
		}else{
			prop_cmd.x = +1.0;
		}
	}

	prop_cmd_history.push_front(prop_cmd);
	if(prop_cmd_history.size() >= HISTORY_SIZE){
		prop_cmd_history.pop_back();
	}
	
	// Filtering by voting
	int points_for_prop_cmd = 0;
	for(auto& pch : prop_cmd_history){
		if(pch == prop_cmd){
			points_for_prop_cmd++;
		}
	}
	float prop_establishment_percents =
		100.0*points_for_prop_cmd/prop_cmd_history.size();
	if(prop_establishment_percents > ESTABLISHMENT_TAKEOVER_PERCENTS){
		established_cmd = prop_cmd;
	}
	
	cmd = established_cmd;
	
#if 0
	DEBUG(best->pt);
	DEBUG(d);
	DEBUG(size);
	DEBUG(prop_cmd);
	DEBUG(cmd);
	DEBUG(prop_establishment_percents);
#endif

}
