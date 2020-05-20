// 展开int2str
#include <bits/stdc++.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/mman.h>

using namespace std;

#if RECORD_TIME
#include <chrono>
class TicToc{
  public:
    TicToc(){
        tic();
    }

    void tic(){
        start = std::chrono::system_clock::now();
    }

    double toc(){
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count(); // seconds
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};
#endif

/***************************************************************************
 *                       
 *                          常量参数
 * 
 **************************************************************************/ 
static const int kMaxVertexSize = 1600000;
static const int kMaxEdgeSize = 2000000;
static const int kMaxSubEdgeSize=1010000;
static const int kThreadCount = 4;
static const int kOutputBufferSize = 77*20000000;
static const int kAllCyclesSize[8] = {0, 0, 0, 3*4000000, 4*4000000, 5*7000000, 6*1000000, 7*12000000};

const int c_BlockSize = 512;
int task_to_thread[kMaxVertexSize]{-1};

namespace std
{
class threadpool
{
    using Task =std::shared_ptr<std::packaged_task<void(int)>>;
    std::vector<std::thread> pool;
	int poolSize;
	std::vector<Task> tasks;
        std::vector<int> id{0,1,2,3,4,5,6,7,8};
	std::atomic<int> killed_points;
	int total_points;
public:
    inline threadpool(unsigned short size = 4) : killed_points{0} , poolSize{size} , total_points{0},tasks(4000000){
	}
	void run()
	{
        for (int i= 0; i < poolSize; ++i)
        {   
            pool.emplace_back(
                [this,i]
                { 
		    Task task;
                    while(1)
                    {
			int tmp = killed_points;
			if(killed_points.compare_exchange_weak(tmp,tmp+1))
			{
				if(tmp >= total_points) return;
				task = std::move(this->tasks[tmp]); // 取一个 task
				(*task)(id[i]);
			}
                    }
                }
            );
        }
    }
    inline ~threadpool()
    {
        for (std::thread& thread : pool) {
            if(thread.joinable())
                thread.join(); 
        }	
    }

public:
    template<class F, class... Args>
    auto commit(F&& f, Args&&... args) ->std::future<void>
    {
        auto task = std::make_shared<std::packaged_task<void(int)>>(
            std::bind(std::forward<F>(f), std::placeholders::_1,std::forward<Args>(args)...)
        );    
        std::future<void> future = task->get_future();
        tasks[total_points++] = task;
        return future;
    }
};
}

/***************************************************************************
 *                       
 *                           数据结构和宏
 * 
 **************************************************************************/ 
struct EdgeInfo{
    const char *str = 0;  // from的字符串指针
    int len = 0;    // from的字符长度
    int from = 0;
    int to = 0;
    int weight = 0;
};

struct Edge{
    int to = 0;
    int weight = 0;
};

struct VertexInfo{
    Edge *start = 0;
    int nbr_size = 0;
}__attribute__ ((aligned (16)));

struct VertexMapInfo{
    const char* str = 0;
    int len = 0;
    int vertex = 0;
};

struct StackMapInfo{
    int nbr7 = 0;
    int nbr6 = 0;
    int w7 = 0;
    int w5 = 0;
};

struct CycleInfoNode{
    int nint = 0;       // vertex num
    int begin = 0;      // begin index
}; 

#define LIKELY(x)  __builtin_expect((x), 1)
#define UNLIKELY(x)  __builtin_expect((x), 0)
/***************************************************************************
 *                       
 *                           全局变量
 * 
 **************************************************************************/ 

threadpool fire{4};
char g_table[kThreadCount][kMaxEdgeSize * 8];
EdgeInfo g_edge_info_unordered[kThreadCount][kMaxEdgeSize];   // 22M / thread
EdgeInfo g_inv_edge_info_unordered[kThreadCount][kMaxEdgeSize];
EdgeInfo merge_sub_edge[2][kMaxSubEdgeSize];
EdgeInfo g_edge_info_ordered[kMaxEdgeSize];
int t_edge_info_size[kThreadCount]{0};
int g_edge_info_size = 0;

VertexMapInfo g_vertex_map[kMaxVertexSize];

int g_graph_sizes = 0;
Edge g_edges[kMaxEdgeSize];                     // 15M 
VertexInfo g_vertex_info[kMaxVertexSize];       // 30M
Edge g_inv_edges[kMaxEdgeSize];                 
VertexInfo g_inv_vertex_info[kMaxVertexSize];

vector<vector<StackMapInfo> > g_stack_map[kThreadCount];

int g_mark_array[kThreadCount][kMaxVertexSize];

int g_indegree[kMaxVertexSize]{0};
bool g_instack[kThreadCount][kMaxVertexSize]{0};

vector<vector<CycleInfoNode> > g_cycles_map;            // g_cycles_map[vertex_id][depth]
vector<vector<int> > g_all_cycles[kThreadCount];   // g_all_cycles[thread_id][depth][vertex_id_in_cycles]
int g_cycle_num[kThreadCount][16]{0};

char* g_output_buf[kThreadCount]{nullptr};
int g_output_size[kThreadCount][16];
char *g_output_ptr[kThreadCount][8];

int g_test_file = -1;
FILE *g_reslt_file = nullptr;

inline int eolOffset(const char* addr, int range){
    const char EOL = '\n';
    int i = 0;    
    while (*(addr - i) != EOL){
        ++i;
        if (i >= range)
            return 0;
    }
    return (i - 1);
}

void loadTestDataThread(int thread_id, const char *start, const char *end){
    EdgeInfo *edge_info_thread = g_edge_info_unordered[thread_id];    
    EdgeInfo *inv_edge_info_thread=g_inv_edge_info_unordered[thread_id];
    char *table = g_table[thread_id];
    int edge_info_size = 0;
    while(start != end){
        int from = 0, to = 0, weight = 0;
        const char *str = start;
        while(*start != ','){
            from = (from << 3) + (from << 1) + (*start++ - '0');
        }
		++start;
        int len = start - str;  // 包括逗号
        while(*start != ','){
            to = (to << 3) + (to << 1) + (*start++ - '0');
        }
        ++start;
        while(*start != '\n'){
            if (isdigit(*start)){
                weight = (weight << 3) + (weight << 1) + (*start - '0');
            }
            ++start;
        }
        ++start;
        if (weight == 0){
            continue;
        }
        memcpy(table, str, len);
        (*(edge_info_thread+edge_info_size)).str = table;
        (*(edge_info_thread+edge_info_size)).len = len;
        (*(edge_info_thread+edge_info_size)).from = from;
        (*(edge_info_thread+edge_info_size)).to = to;
        (*(edge_info_thread+edge_info_size)).weight = weight;
        table += len;
        ++edge_info_size;
    }
    t_edge_info_size[thread_id] = edge_info_size;
    memcpy(inv_edge_info_thread,edge_info_thread,sizeof(EdgeInfo)*edge_info_size);
    sort(edge_info_thread,edge_info_thread+edge_info_size,[](const EdgeInfo &lhs, const EdgeInfo &rhs){
                                                           return lhs.from < rhs.from;});
    sort(inv_edge_info_thread,inv_edge_info_thread+edge_info_size,[](const EdgeInfo &lhs, const EdgeInfo &rhs){
                                                           return lhs.to < rhs.to;});
}

inline bool cmpEdge(const Edge &lhs, const Edge &rhs){
    return lhs.to < rhs.to;
}

void topoSort(){
    std::queue<int> Q;
    for (int vertex = 0; vertex < g_graph_sizes; ++vertex){//可以优化
        if (0 == g_indegree[vertex]){
            Q.push(vertex);
        }
    }
    while(!Q.empty()){
        int vertex = Q.front();
        Q.pop();
        for(int i=0;i<g_vertex_info[vertex].nbr_size;i++)
        {
            Edge * edge=g_vertex_info[vertex].start+i;
            int v=edge->to;
            if (0 == --g_indegree[v]){
                Q.push(v);
            }
        }
        g_vertex_info[vertex].nbr_size=0;
    }
}

void loadTestData(){
#if RECORD_TIME
    TicToc tt1;
#endif
    struct stat statbuf = {0};
    fstat(g_test_file, &statbuf);
    int srclen = statbuf.st_size;
    const char *g_src = (const char *)mmap(0, srclen, PROT_READ, MAP_PRIVATE, g_test_file, 0);

    int part_len = srclen / kThreadCount + 1;   
    const char *joint_addr[kThreadCount+1];
    joint_addr[0] = g_src;
    for (int i = 1; i < kThreadCount; ++i)
    {
        const char* addr = g_src + part_len * i;
        joint_addr[i] = addr - eolOffset(addr, part_len);
    }
    joint_addr[kThreadCount] = g_src + srclen;

    vector<thread *> thread_list(kThreadCount);
    for (int thread_id = 0; thread_id < kThreadCount; ++thread_id){
        thread_list[thread_id] = new thread(loadTestDataThread, thread_id, joint_addr[thread_id], joint_addr[thread_id+1]);
    }
#if RECORD_TIME
    double t1 = tt1.toc();
    TicToc tt2;
#endif
    // build id map
    for (int thread_id = 0; thread_id < kThreadCount; ++thread_id){
        thread_list[thread_id]->join();
        delete thread_list[thread_id];

        g_edge_info_size += t_edge_info_size[thread_id];
    }
#if RECORD_TIME
	TicToc tt_merge;
#endif
    merge(g_edge_info_unordered[0],g_edge_info_unordered[0]+t_edge_info_size[0],g_edge_info_unordered[1],g_edge_info_unordered[1]+t_edge_info_size[1],merge_sub_edge[0],[](const EdgeInfo &lhs, const EdgeInfo &rhs){
                                                           return lhs.from < rhs.from;});
    merge(g_edge_info_unordered[2],g_edge_info_unordered[2]+t_edge_info_size[2],g_edge_info_unordered[3],g_edge_info_unordered[3]+t_edge_info_size[3],merge_sub_edge[1],[](const EdgeInfo &lhs, const EdgeInfo &rhs){
                                                           return lhs.from < rhs.from;});

    merge(merge_sub_edge[0],merge_sub_edge[0]+(t_edge_info_size[0]+t_edge_info_size[1]),merge_sub_edge[1],merge_sub_edge[1]+(t_edge_info_size[2]+t_edge_info_size[3]),g_edge_info_ordered,[](const EdgeInfo &lhs, const EdgeInfo &rhs){
                                                           return lhs.from < rhs.from;});
    
    
#if RECORD_TIME
	double t_merge=tt_merge.toc();
#endif
    // build graph
	int prev_from = g_edge_info_ordered[0].from;
    int from_id = 0;
    int cur_start = 0, cur_nbr_size = 1;
    g_vertex_info[0].start = g_edges + cur_start;
    g_vertex_map[0].str = g_edge_info_ordered[0].str;
    g_vertex_map[0].len = g_edge_info_ordered[0].len;
    g_vertex_map[0].vertex = prev_from;
    for (int i = 1; i < g_edge_info_size; ++i){
        if (g_edge_info_ordered[i].from != prev_from){
            g_vertex_info[from_id].nbr_size = cur_nbr_size;
            cur_start += cur_nbr_size;
            cur_nbr_size = 1;
            prev_from = g_edge_info_ordered[i].from;
            g_vertex_map[++from_id].str = g_edge_info_ordered[i].str;
            g_vertex_map[from_id].len = g_edge_info_ordered[i].len;
            g_vertex_map[from_id].vertex = prev_from;
            g_vertex_info[from_id].start = g_edges + cur_start;
            continue;
        }
        ++cur_nbr_size;
    }
    g_vertex_info[from_id].nbr_size = cur_nbr_size;
    g_graph_sizes = from_id+1;
#if RECORD_TIME
	TicToc tt_map_id;
#endif
    unordered_map<int, int> g_inv_vertex_map;       // g_inv_vertex_map[vertex] = vertex_id
    g_inv_vertex_map.reserve(g_graph_sizes);
    for (int i = 0; i < g_graph_sizes; ++i){
        g_inv_vertex_map[g_vertex_map[i].vertex] = i;
    }
#if RECORD_TIME
	double t_map_id=tt_map_id.toc();
#endif
    for (int i = 0; i < g_edge_info_size; ++i){
        if(g_inv_vertex_map.count(g_edge_info_ordered[i].to)){
            g_edges[i].to = g_inv_vertex_map[g_edge_info_ordered[i].to];
            g_edges[i].weight = g_edge_info_ordered[i].weight;
        }
        else{
            g_inv_vertex_map[g_edge_info_ordered[i].to]= ++from_id;
            g_edges[i].to = from_id;
            g_edges[i].weight = g_edge_info_ordered[i].weight;
        }
    }
    g_graph_sizes=from_id+1;
#if RECORD_TIME
	TicToc tt_inv_sort;
#endif
    // build inv graph
    merge(g_inv_edge_info_unordered[0],g_inv_edge_info_unordered[0]+t_edge_info_size[0],g_inv_edge_info_unordered[1],g_inv_edge_info_unordered[1]+t_edge_info_size[1],merge_sub_edge[0],[](const EdgeInfo &lhs, const EdgeInfo &rhs){
                                                           return lhs.to < rhs.to;});
    merge(g_inv_edge_info_unordered[2],g_inv_edge_info_unordered[2]+t_edge_info_size[2],g_inv_edge_info_unordered[3],g_inv_edge_info_unordered[3]+t_edge_info_size[3],merge_sub_edge[1],[](const EdgeInfo &lhs, const EdgeInfo &rhs){
                                                           return lhs.to < rhs.to;});

    merge(merge_sub_edge[0],merge_sub_edge[0]+(t_edge_info_size[0]+t_edge_info_size[1]),merge_sub_edge[1],merge_sub_edge[1]+(t_edge_info_size[2]+t_edge_info_size[3]),g_edge_info_ordered,[](const EdgeInfo &lhs, const EdgeInfo &rhs){
                                                           return lhs.to < rhs.to;});
#if RECORD_TIME
    double t_sort=tt_inv_sort.toc();
#endif                                                              
    int prev_to = g_edge_info_ordered[0].to;
    cur_start = 0, cur_nbr_size = 1;
    int start_id=g_inv_vertex_map[g_edge_info_ordered[0].to];
    g_inv_vertex_info[start_id].start = g_inv_edges;
    for (int i = 1; i < g_edge_info_size; ++i){
        if (g_edge_info_ordered[i].to != prev_to){
            int to_id=g_inv_vertex_map[prev_to];
            g_inv_vertex_info[to_id].nbr_size = cur_nbr_size;
            g_indegree[to_id]=cur_nbr_size;
            cur_start += cur_nbr_size;
            cur_nbr_size = 1;
            prev_to = g_edge_info_ordered[i].to;
            int next_id=g_inv_vertex_map[g_edge_info_ordered[i].to];
            g_inv_vertex_info[next_id].start = g_inv_edges + cur_start;
            continue;
        }
        ++cur_nbr_size;
    }
    int end_id=g_inv_vertex_map[g_edge_info_ordered[g_edge_info_size-1].to];
    g_inv_vertex_info[end_id].nbr_size = cur_nbr_size;
	g_indegree[end_id]=cur_nbr_size;
    for (int i = 0; i < g_edge_info_size; ++i){
        g_inv_edges[i].to = g_inv_vertex_map[g_edge_info_ordered[i].from];
        g_inv_edges[i].weight = g_edge_info_ordered[i].weight;
    }
#if RECORD_TIME
	TicToc tt_sort2;
#endif
	topoSort();
     for (int i = 0; i < g_graph_sizes; ++i){
         sort(g_vertex_info[i].start, g_vertex_info[i].start+g_vertex_info[i].nbr_size, cmpEdge);
         sort(g_inv_vertex_info[i].start, g_inv_vertex_info[i].start+g_inv_vertex_info[i].nbr_size, cmpEdge);
     }

#if RECORD_TIME
	double t_sort2=tt_sort2.toc();
    double t2 = tt2.toc();
#endif

#if RECORD_TIME
    cout << "\tmmap: " << t1 << endl;
    cout << "\tbuild graph: " << t2 << endl;
	cout<<"\tmerge  sort: "<<t_sort<<endl;
	cout<<"\tmap id: "<<t_map_id<<endl;
	cout<<"\t neighbour sort"<<t_sort2<<endl;
    cout<<"\t merge inv sort"<<t_merge<<endl;
#endif
}

inline bool checkWeight(int w1, int w2){
    return w2 <= 3l*w1 && w1 <= 5l*w2;
}

inline bool rejectWeight(int w1, int w2){
    return w2 > 3l*w1 || w1 > 5l*w2;
}

void iterationInv(const int start_vertex, const int thread_id){
    bool *instack = g_instack[thread_id];
    int *mark_array_thread = g_mark_array[thread_id];
    vector<vector<StackMapInfo> > &stack_map_thread = g_stack_map[thread_id];
    
    instack[start_vertex] = true;
    Edge *pend1 = g_inv_vertex_info[start_vertex].start + g_inv_vertex_info[start_vertex].nbr_size;
    for (Edge *p1 = g_inv_vertex_info[start_vertex].start; p1 != pend1; ++p1){
        int nbr1 = (*p1).to;
        int w1 = (*p1).weight;
        if (nbr1 <= start_vertex){
            continue;
        }
        instack[nbr1] = true;
        Edge *pend2 = g_inv_vertex_info[nbr1].start + g_inv_vertex_info[nbr1].nbr_size;
        for(Edge *p2 = g_inv_vertex_info[nbr1].start; p2 != pend2; ++p2){
            int nbr2 = (*p2).to;
            int w2 = (*p2).weight;
            if (nbr2 <= start_vertex || instack[nbr2] || rejectWeight(w2, w1)){
                continue;
            }
            instack[nbr2] = true;
            Edge *pend3 = g_inv_vertex_info[nbr2].start + g_inv_vertex_info[nbr2].nbr_size;
            for(Edge *p3 = g_inv_vertex_info[nbr2].start; p3 != pend3; ++p3){
                int nbr3 = (*p3).to;
                int w3 = (*p3).weight;
                if (nbr3 <= start_vertex || instack[nbr3] || rejectWeight(w3, w2)){
                    continue;
                }
                StackMapInfo smi;
                smi.nbr6 = nbr2;
                smi.nbr7 = nbr1;
                smi.w7 = w1;
                smi.w5 = w3;
                if (mark_array_thread[nbr3] != start_vertex){
                    mark_array_thread[nbr3] = start_vertex;
                    stack_map_thread[nbr3].clear();
                }
                auto bgn = stack_map_thread[nbr3].begin();
                auto end = stack_map_thread[nbr3].end();
                for ( ; bgn != end; ++bgn){
                    if (smi.nbr6 < (*bgn).nbr6){
                        break;
                    }
                }
                stack_map_thread[nbr3].insert(bgn, smi);
            }
            instack[nbr2] = false;
        }
        instack[nbr1] = false;
    }
}

void iteration(const int start_vertex, const int thread_id, int *cycle_num_thread){
    bool *instack = g_instack[thread_id];
    vector<vector<StackMapInfo> > &stack_map_thread = g_stack_map[thread_id];
    vector<vector<int> > &all_cycles_thread = g_all_cycles[thread_id];
    // int *cycle_num_thread = g_cycle_num[thread_id];
    vector<CycleInfoNode> &ci = g_cycles_map[start_vertex];
    int *mark_array_thread = g_mark_array[thread_id];
    Edge *pend1 = g_vertex_info[start_vertex].start + g_vertex_info[start_vertex].nbr_size;
    for(Edge *p1 = g_vertex_info[start_vertex].start; p1 != pend1; ++p1){    // layer 1
        int nbr1 = (*p1).to;
        int w1 = (*p1).weight;
        if(nbr1 <= start_vertex){
            continue;
        }
        instack[nbr1] = true;
        if(mark_array_thread[nbr1] == start_vertex){
            for(const StackMapInfo &smi: stack_map_thread[nbr1]){ // cycles of length 4
                if (checkWeight(w1, smi.w5) && checkWeight(smi.w7, w1)){
                    const int cycle_len = 4;
                    if (ci[cycle_len].nint == 0){
                        ci[cycle_len].begin = all_cycles_thread[cycle_len].size(); 
                    }
                    all_cycles_thread[cycle_len].push_back(start_vertex);
                    all_cycles_thread[cycle_len].push_back(nbr1);
                    all_cycles_thread[cycle_len].push_back(smi.nbr6);
                    all_cycles_thread[cycle_len].push_back(smi.nbr7);
                    ++cycle_num_thread[cycle_len];
                    ci[cycle_len].nint += cycle_len;
                }
            }
        }
        Edge *pend2 = g_vertex_info[nbr1].start + g_vertex_info[nbr1].nbr_size;
        for(Edge *p2 = g_vertex_info[nbr1].start; p2 != pend2; ++p2){
            int nbr2 = (*p2).to;
            int w2 = (*p2).weight;
            if(nbr2 <= start_vertex || nbr1 == nbr2 || rejectWeight(w1, w2)){
                continue;
            }
            instack[nbr2] = true;
            if(mark_array_thread[nbr2]==start_vertex){
                for(const StackMapInfo &smi: stack_map_thread[nbr2]){    // cycles of length 5
                    if (instack[smi.nbr6] == false && instack[smi.nbr7] == false &&
                        checkWeight(w2, smi.w5) && checkWeight(smi.w7, w1)){
                        const int cycle_len = 5;
                        if (ci[cycle_len].nint == 0){
                            ci[cycle_len].begin = all_cycles_thread[cycle_len].size(); 
                        }
                        all_cycles_thread[cycle_len].push_back(start_vertex);
                        all_cycles_thread[cycle_len].push_back(nbr1);
                        all_cycles_thread[cycle_len].push_back(nbr2);
                        all_cycles_thread[cycle_len].push_back(smi.nbr6);
                        all_cycles_thread[cycle_len].push_back(smi.nbr7);
                        ++cycle_num_thread[cycle_len];
                        ci[cycle_len].nint += cycle_len;
                    }
                }
            }
            Edge *pend3 = g_vertex_info[nbr2].start + g_vertex_info[nbr2].nbr_size;
            for(Edge *p3 = g_vertex_info[nbr2].start; p3 != pend3; ++p3){
                int nbr3 = (*p3).to;
                int w3 = (*p3).weight;
                if(nbr3 < start_vertex || (nbr3 != start_vertex && instack[nbr3]) || rejectWeight(w2, w3)){
                        continue;
                }
                else if (nbr3 == start_vertex){             // cycles of length 3
                    if (checkWeight(w3, w1)){
                        const int cycle_len = 3;    
                        if (ci[cycle_len].nint == 0){
                            ci[cycle_len].begin = all_cycles_thread[cycle_len].size(); 
                        }
                        all_cycles_thread[cycle_len].push_back(start_vertex);
                        all_cycles_thread[cycle_len].push_back(nbr1);
                        all_cycles_thread[cycle_len].push_back(nbr2);
                        ++cycle_num_thread[cycle_len];
                        ci[cycle_len].nint += cycle_len;
                    }
                    continue;
                }
                instack[nbr3] = true;
                if(mark_array_thread[nbr3] == start_vertex){
                    for (const StackMapInfo &smi: stack_map_thread[nbr3]){ // cycles of length 6
                        if (instack[smi.nbr6] == false && instack[smi.nbr7] == false &&
                            checkWeight(w3, smi.w5) && checkWeight(smi.w7, w1)){
                            const int cycle_len = 6;
                            if (ci[cycle_len].nint == 0){
                                ci[cycle_len].begin = all_cycles_thread[cycle_len].size(); 
                            }
                            all_cycles_thread[cycle_len].push_back(start_vertex);
                            all_cycles_thread[cycle_len].push_back(nbr1);
                            all_cycles_thread[cycle_len].push_back(nbr2);
                            all_cycles_thread[cycle_len].push_back(nbr3);
                            all_cycles_thread[cycle_len].push_back(smi.nbr6);
                            all_cycles_thread[cycle_len].push_back(smi.nbr7);
                            ++cycle_num_thread[cycle_len];
                            ci[cycle_len].nint += cycle_len;
                        }
                    }
                }
                Edge *pend4 = g_vertex_info[nbr3].start + g_vertex_info[nbr3].nbr_size;
                for(Edge *p4 = g_vertex_info[nbr3].start; p4 != pend4; ++p4){
                    int nbr4 = (*p4).to;
                    int w4 = (*p4).weight;
                    if(nbr4 <= start_vertex || instack[nbr4] || rejectWeight(w3, w4)){
                        continue;
                    }
                    if (UNLIKELY(mark_array_thread[nbr4] == start_vertex)){
                        for (const StackMapInfo &smi: stack_map_thread[nbr4]){     // cycles of length 7
                            if (instack[smi.nbr6] == false && instack[smi.nbr7] == false &&
                                smi.nbr6 != nbr4 && smi.nbr7 != nbr4 &&
                                checkWeight(w4, smi.w5) && checkWeight(smi.w7, w1)){
                                const int cycle_len = 7;
                                if (ci[cycle_len].nint == 0){
                                    ci[cycle_len].begin = all_cycles_thread[cycle_len].size(); 
                                }
                                all_cycles_thread[cycle_len].push_back(start_vertex);
                                all_cycles_thread[cycle_len].push_back(nbr1);
                                all_cycles_thread[cycle_len].push_back(nbr2);
                                all_cycles_thread[cycle_len].push_back(nbr3);
                                all_cycles_thread[cycle_len].push_back(nbr4);
                                all_cycles_thread[cycle_len].push_back(smi.nbr6);
                                all_cycles_thread[cycle_len].push_back(smi.nbr7);
                                ++cycle_num_thread[cycle_len];
                                ci[cycle_len].nint += cycle_len;
                            }
                        }
                    }
                }
                instack[nbr3] = false;
            }
            instack[nbr2] = false;
        }
        instack[nbr1] = false;
    }
    instack[start_vertex] = false;
}

char* int2strCommon(int x, char *str){
    int remain = 0;
    int div_x  = 0;
    int le_zero= 0;

    *str-- = ',';
    if (x == 0){
        *str-- = '0';
    }
    else{
        while (x > 0) {
            div_x  = x / 10;
            remain = x - ((div_x<<3)+(div_x<<1));  // x - div_x * 10
            x      = div_x;

            *str-- = remain+'0';
        }
    }
    return str;
}

void int2strThread(const int thread_id, int start, int end){
    char *ptr = g_output_buf[thread_id];
    int ids[8];
    // 3
    g_output_ptr[thread_id][3] = ptr;
    for (int vtx = start; vtx < end; ++vtx){
        if (!g_cycles_map[vtx].empty()){
            auto src = g_all_cycles[task_to_thread[vtx/c_BlockSize]][3].data() + g_cycles_map[vtx][3].begin;
            int nint = g_cycles_map[vtx][3].nint;
            for(int i = 0; i < nint; i += 3){
                memcpy(ids, src + i, 12);  
                char *ptr1 = ptr + g_vertex_map[ids[0]].len;
                char *ptr2 = ptr1 + g_vertex_map[ids[1]].len;
                memcpy(ptr, g_vertex_map[ids[0]].str, g_vertex_map[ids[0]].len);
                memcpy(ptr1, g_vertex_map[ids[1]].str, g_vertex_map[ids[1]].len);
                memcpy(ptr2, g_vertex_map[ids[2]].str, g_vertex_map[ids[2]].len);
                ptr = ptr2 + g_vertex_map[ids[2]].len;
                *(ptr-1) = '\n';
            }
        }
    }
    g_output_size[thread_id][3] = ptr - g_output_ptr[thread_id][3];

    // 4
    g_output_ptr[thread_id][4] = ptr;
    for (int vtx = start; vtx < end; ++vtx){
        if (!g_cycles_map[vtx].empty()){
            auto src = g_all_cycles[task_to_thread[vtx/c_BlockSize]][4].data() + g_cycles_map[vtx][4].begin;
            int nint = g_cycles_map[vtx][4].nint;
            for(int i = 0; i < nint; i += 4){
                memcpy(ids, src + i, 16);
                char *ptr1 = ptr + g_vertex_map[ids[0]].len;
                char *ptr2 = ptr1 + g_vertex_map[ids[1]].len;
                char *ptr3 = ptr2 + g_vertex_map[ids[2]].len;
                memcpy(ptr, g_vertex_map[ids[0]].str, g_vertex_map[ids[0]].len);
                memcpy(ptr1, g_vertex_map[ids[1]].str, g_vertex_map[ids[1]].len);
                memcpy(ptr2, g_vertex_map[ids[2]].str, g_vertex_map[ids[2]].len);
                memcpy(ptr3, g_vertex_map[ids[3]].str, g_vertex_map[ids[3]].len);
                ptr = ptr3 + g_vertex_map[ids[3]].len;
                *(ptr-1) = '\n';
            }
        }
    }
    g_output_size[thread_id][4] = ptr - g_output_ptr[thread_id][4];

    // 5
    g_output_ptr[thread_id][5] = ptr;
    for (int vtx = start; vtx < end; ++vtx){
        if (!g_cycles_map[vtx].empty()){
            auto src = g_all_cycles[task_to_thread[vtx/c_BlockSize]][5].data() + g_cycles_map[vtx][5].begin;
            int nint = g_cycles_map[vtx][5].nint;
            for(int i = 0; i < nint; i += 5){
                memcpy(ids, src + i, 20);
                char *ptr1 = ptr + g_vertex_map[ids[0]].len;
                char *ptr2 = ptr1 + g_vertex_map[ids[1]].len;
                char *ptr3 = ptr2 + g_vertex_map[ids[2]].len;
                char *ptr4 = ptr3 + g_vertex_map[ids[3]].len;
                memcpy(ptr, g_vertex_map[ids[0]].str, g_vertex_map[ids[0]].len);
                memcpy(ptr1, g_vertex_map[ids[1]].str, g_vertex_map[ids[1]].len);
                memcpy(ptr2, g_vertex_map[ids[2]].str, g_vertex_map[ids[2]].len);
                memcpy(ptr3, g_vertex_map[ids[3]].str, g_vertex_map[ids[3]].len);
                memcpy(ptr4, g_vertex_map[ids[4]].str, g_vertex_map[ids[4]].len);
                ptr = ptr4 + g_vertex_map[ids[4]].len;
                *(ptr-1) = '\n';
            }
        }
    }
    g_output_size[thread_id][5] = ptr - g_output_ptr[thread_id][5];
    // 6
    g_output_ptr[thread_id][6] = ptr;
    for (int vtx = start; vtx < end; ++vtx){
        if (!g_cycles_map[vtx].empty()){
            auto src = g_all_cycles[task_to_thread[vtx/c_BlockSize]][6].data() + g_cycles_map[vtx][6].begin;
            int nint = g_cycles_map[vtx][6].nint;
            for(int i = 0; i < nint; i += 6){
                memcpy(ids, src + i, 24);
                char *ptr1 = ptr + g_vertex_map[ids[0]].len;
                char *ptr2 = ptr1 + g_vertex_map[ids[1]].len;
                char *ptr3 = ptr2 + g_vertex_map[ids[2]].len;
                char *ptr4 = ptr3 + g_vertex_map[ids[3]].len;
                char *ptr5 = ptr4 + g_vertex_map[ids[4]].len;
                memcpy(ptr, g_vertex_map[ids[0]].str, g_vertex_map[ids[0]].len);
                memcpy(ptr1, g_vertex_map[ids[1]].str, g_vertex_map[ids[1]].len);
                memcpy(ptr2, g_vertex_map[ids[2]].str, g_vertex_map[ids[2]].len);
                memcpy(ptr3, g_vertex_map[ids[3]].str, g_vertex_map[ids[3]].len);
                memcpy(ptr4, g_vertex_map[ids[4]].str, g_vertex_map[ids[4]].len);
                memcpy(ptr5, g_vertex_map[ids[5]].str, g_vertex_map[ids[5]].len);
                ptr = ptr5 + g_vertex_map[ids[5]].len;
                *(ptr-1) = '\n';
            }
        }
    }
    g_output_size[thread_id][6] = ptr - g_output_ptr[thread_id][6];

    // 7
    g_output_ptr[thread_id][7] = ptr;
    for (int vtx = start; vtx < end; ++vtx){
        if (!g_cycles_map[vtx].empty()){
            auto src = g_all_cycles[task_to_thread[vtx/c_BlockSize]][7].data() + g_cycles_map[vtx][7].begin;
            int nint = g_cycles_map[vtx][7].nint;
            for(int i = 0; i < nint; i += 7){
                memcpy(ids, src + i, 28);
                char *ptr1 = ptr + g_vertex_map[ids[0]].len;
                char *ptr2 = ptr1 + g_vertex_map[ids[1]].len;
                char *ptr3 = ptr2 + g_vertex_map[ids[2]].len;
                char *ptr4 = ptr3 + g_vertex_map[ids[3]].len;
                char *ptr5 = ptr4 + g_vertex_map[ids[4]].len;
                char *ptr6 = ptr5 + g_vertex_map[ids[5]].len;
                memcpy(ptr, g_vertex_map[ids[0]].str, g_vertex_map[ids[0]].len);
                memcpy(ptr1, g_vertex_map[ids[1]].str, g_vertex_map[ids[1]].len);
                memcpy(ptr2, g_vertex_map[ids[2]].str, g_vertex_map[ids[2]].len);
                memcpy(ptr3, g_vertex_map[ids[3]].str, g_vertex_map[ids[3]].len);
                memcpy(ptr4, g_vertex_map[ids[4]].str, g_vertex_map[ids[4]].len);
                memcpy(ptr5, g_vertex_map[ids[5]].str, g_vertex_map[ids[5]].len);
                memcpy(ptr6, g_vertex_map[ids[6]].str, g_vertex_map[ids[6]].len);
                ptr = ptr6 + g_vertex_map[ids[6]].len;
                *(ptr-1) = '\n';
            }
        }
    }
    g_output_size[thread_id][7] = ptr - g_output_ptr[thread_id][7];
}

void storeReslt(){
#if RECORD_TIME
    TicToc tt1;
#endif
    int all_sizes = 0;
	int nint_sizes = 0;
    for(int i=0; i < kThreadCount; ++i){
        for (int j = 3; j <= 7; ++j){
			nint_sizes += g_cycle_num[i][j] * j;
            all_sizes += g_cycle_num[i][j];
        }
    }
    char digit[16];
    char *end = digit + 15;
    char *str = int2strCommon(all_sizes, end);
    int len = end - str;
    *end = '\n';
    fwrite(str+1, len, 1, g_reslt_file);
    int part_size = nint_sizes / 4;
    int cur_boundary = part_size;
    int acc_size = 0;
    int start_vertex[5] = {0, 0, 0, 0, g_graph_sizes};
    int tmp_idx = 1;
    for (int vtx = 0; vtx < g_graph_sizes; ++vtx){
        if (!g_cycles_map[vtx].empty()){
            for (int cl = 3; cl <= 7; ++cl){
                acc_size += g_cycles_map[vtx][cl].nint;
            }
        }
        if (acc_size >= cur_boundary){
            start_vertex[tmp_idx++] = vtx;
            cur_boundary += part_size;
            if (tmp_idx == 4){
                break;
            }
        }
    }
    std::thread *thread_list[kThreadCount];
    for(int thread_id=0;thread_id<kThreadCount;++thread_id){
        thread_list[thread_id] = new std::thread(int2strThread, thread_id, start_vertex[thread_id], start_vertex[thread_id+1]);
    }
    for (int thread_id = 0; thread_id < kThreadCount; ++thread_id){
        thread_list[thread_id]->join();
    }
#if RECORD_TIME
    double t1 = tt1.toc();
    TicToc tt2;
#endif
    for (int i = 3; i <= 7; ++i){
        for (int thread_id = 0; thread_id < kThreadCount; ++thread_id){
            fwrite(g_output_ptr[thread_id][i], g_output_size[thread_id][i], 1, g_reslt_file);
        }
    }
#if RECORD_TIME
    double t2 = tt2.toc();
    TicToc tt3;
#endif
    fclose(g_reslt_file);
#if RECORD_TIME
    double t3 = tt3.toc();
    cout << "\tint2str: " << t1 << endl;
    cout << "\tfwrite: " << t2 << endl;
    cout << "\tdisk IO: " << t3 << endl;
#endif
}

/************************************************/
void work(int thread_id,int task_id){
    int cycle_num_thread[8];
    memcpy(cycle_num_thread+3, g_cycle_num[thread_id]+3, 20);

    task_to_thread[task_id] = thread_id;
    int start = task_id*c_BlockSize;
    int end = min((task_id+1)*c_BlockSize,g_graph_sizes);
    for(int start_vertex = start; start_vertex < end; start_vertex++){
        if (g_vertex_info[start_vertex].nbr_size != 0){
            g_cycles_map[start_vertex].resize(8);
            iterationInv(start_vertex, thread_id);
            iteration(start_vertex, thread_id, cycle_num_thread);
        }
    }
    memcpy(g_cycle_num[thread_id]+3, cycle_num_thread+3, 20);
}

int main(int argc, char **argv)
{
    #if TEST
    if (argc != 2){
        cerr << "./baseline test_file_path" << endl;
        exit(0);
    }
    string kTestFile(argv[1]);
    const string kResltFile = "./my_result.txt";
    #else
    const string kTestFile = "/data/test_data.txt";
    const string kResltFile = "/projects/student/result.txt";
    #endif

    g_test_file = open(kTestFile.c_str(), O_RDONLY);
    g_reslt_file = fopen(kResltFile.c_str(), "w");
  
#if RECORD_TIME
    TicToc tt1;
#endif
    loadTestData();
#if RECORD_TIME
    double t1 = tt1.toc();
    cout << "loadTestData: " << t1 << endl;
    TicToc tt2;
#endif
    for(int i = 0; i < kThreadCount; i++)
	g_output_buf[i] = (char*)malloc(kOutputBufferSize*sizeof(char));
    g_cycles_map.resize(g_graph_sizes);
    for(int thread_id = 0; thread_id < kThreadCount; thread_id++){
        g_stack_map[thread_id].resize(g_graph_sizes);
        memset(g_mark_array[thread_id], -1, kMaxVertexSize);
        g_all_cycles[thread_id].resize(8);
        for (int cl = 3; cl <= 7; ++cl){
            g_all_cycles[thread_id][cl].reserve(kAllCyclesSize[cl]);
        }
    }

    int taskNum = (g_graph_sizes+c_BlockSize-1)/c_BlockSize;
    future<void> results[taskNum];  // maybe overflow
    for(int i = 0; i < taskNum; i++)
        results[i] = fire.commit(work,i);
    fire.run();
    for(int i = 0; i < taskNum; i++)
        results[i].get();
    
#if RECORD_TIME
    double t2 = tt2.toc();
    cout << "find cycle: " << t2 << endl;
    TicToc tt3;
#endif
    storeReslt();
#if RECORD_TIME
    double t3 = tt3.toc();
    cout << "storeReslt: " << t3 << endl;
#endif
}