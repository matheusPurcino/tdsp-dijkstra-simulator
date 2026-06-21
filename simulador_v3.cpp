#include <iostream>
#include <vector>
#include <queue>
#include <chrono>
#include <random>
#include <cmath>
#include <iomanip>
#include <numeric>
#include <fstream>
#include <sstream>

using namespace std;

// =================================================================
// 1. ESTRUTURAS DE DADOS (FORWARD STAR)
// =================================================================
const double INF = 1e18;

struct Edge {
    int to;
    double move_cost;
    int light_id; 
};

struct TrafficLight {
    double cycle_time;
    double green_time;
};

struct State {
    double time;
    int u;
    bool operator>(const State& other) const {
        return time > other.time;
    }
};

vector<int> V_start;
vector<Edge> E_array;
vector<TrafficLight> lights;
int total_edges_count = 0;

// =================================================================
// 2. CONSTRUÇÃO DO GRAFO EM GRADE (GRID GRAPH)
// =================================================================
void build_grid_graph(int grid_rows, int grid_cols, double density_factor, mt19937& rng) {
    int num_nodes = grid_rows * grid_cols;
    V_start.assign(num_nodes + 1, 0);
    E_array.clear();
    lights.clear();

    // MOD-01: Parâmetros semafóricos em minutos
    uniform_real_distribution<double> dist_cycle(1.0, 2.0);
    uniform_real_distribution<double> dist_green_frac(0.4, 0.6);
    uniform_real_distribution<double> dist_cost(1.0, 10.0);
    uniform_real_distribution<double> dist_has_light(0.0, 1.0);

    vector<int> node_light_id(num_nodes, -1);
    for (int i = 0; i < num_nodes; i++) {
        if (dist_has_light(rng) < 0.6) {
            double C = dist_cycle(rng);
            double g = C * dist_green_frac(rng);
            lights.push_back({C, g});
            node_light_id[i] = (int)lights.size() - 1;
        }
    }

    auto node_id = [&](int r, int c) { return r * grid_cols + c; };

    vector<vector<Edge>> temp_adj(num_nodes);
    int directions[4][2] = {{-1,0},{1,0},{0,-1},{0,1}};

    for (int r = 0; r < grid_rows; r++) {
        for (int c = 0; c < grid_cols; c++) {
            int u = node_id(r, c);
            for (auto& d : directions) {
                int nr = r + d[0], nc = c + d[1];
                if (nr >= 0 && nr < grid_rows && nc >= 0 && nc < grid_cols) {
                    int v = node_id(nr, nc);
                    double cost = dist_cost(rng) * density_factor;
                    temp_adj[u].push_back({v, cost, node_light_id[v]});
                }
            }
        }
    }

    int edge_counter = 0;
    for (int u = 0; u < num_nodes; u++) {
        V_start[u] = edge_counter;
        for (auto& edge : temp_adj[u]) {
            E_array.push_back(edge);
            edge_counter++;
        }
    }
    V_start[num_nodes] = edge_counter;
    total_edges_count = edge_counter;
}

// =================================================================
// 3. OPERADOR MODULAR DE ATRASO (ModuloWaitTime)
// =================================================================
inline double calc_delay(double t, int light_id) {
    if (light_id == -1) return 0.0;
    const TrafficLight& tl = lights[light_id];
    double phase = fmod(t, tl.cycle_time);

    if (phase < tl.green_time) {
        return 0.0; 
    }
    return tl.cycle_time - phase; 
}

// =================================================================
// 4. ALGORITMOS DE ROTEAMENTO
// =================================================================
vector<double> dijkstra_static(int source, int num_nodes) {
    vector<double> dist(num_nodes, INF);
    priority_queue<State, vector<State>, greater<State>> pq;

    dist[source] = 0;
    pq.push({0, source});

    while (!pq.empty()) {
        auto [t_atual, u] = pq.top();
        pq.pop();

        if (t_atual > dist[u]) continue;

        for (int i = V_start[u]; i < V_start[u + 1]; i++) {
            int v = E_array[i].to;
            double peso = E_array[i].move_cost;

            if (dist[u] + peso < dist[v]) {
                dist[v] = dist[u] + peso;
                pq.push({dist[v], v});
            }
        }
    }
    return dist;
}

vector<double> dijkstra_dynamic(int source, int num_nodes) {
    vector<double> dist(num_nodes, INF);
    priority_queue<State, vector<State>, greater<State>> pq;

    dist[source] = 0;
    pq.push({0, source});

    while (!pq.empty()) {
        auto [t_atual, u] = pq.top();
        pq.pop();

        if (t_atual > dist[u]) continue;

        for (int i = V_start[u]; i < V_start[u + 1]; i++) {
            int v = E_array[i].to;
            double peso = E_array[i].move_cost;
            double atraso = calc_delay(t_atual, E_array[i].light_id);

            double novo_tempo = dist[u] + peso + atraso;

            if (novo_tempo < dist[v]) {
                dist[v] = novo_tempo;
                pq.push({novo_tempo, v});
            }
        }
    }
    return dist;
}

// =================================================================
// 5. EXPERIMENTOS
// =================================================================
void simular_tabela_eficacia(ofstream& fout) {
    auto log = [&](const string& s) { cout << s; fout << s; };
    log("\n=== TABELA II: TEMPO DE VIAGEM VS DENSIDADE ===\n");
    log("Densidade\tEstatico_real(min)\tTDSP(min)\tSubestimacao(%)\n");

    int rows = 224, cols = 224;
    int num_nodes = rows * cols;
    int source = 0;
    int target = num_nodes - 1;

    for (int d = 500; d <= 4000; d += 250) {
        // MOD-02: RNG reset a cada densidade
        mt19937 rng(42); 
        double factor = 1.0 + (d / 1000.0) * 0.5;
        build_grid_graph(rows, cols, factor, rng);

        auto dist_static = dijkstra_static(source, num_nodes);
        auto dist_dynamic = dijkstra_dynamic(source, num_nodes);

        double est = dist_static[target];
        double din = dist_dynamic[target];
        double subest = ((din - est) / din) * 100.0;

        ostringstream oss;
        oss << fixed << setprecision(1);
        oss << d << "\t\t" << est << "\t\t\t" << din << "\t\t" << subest << "%\n";
        log(oss.str());
    }
}

void simular_tabela_cpu(ofstream& fout) {
    auto log = [&](const string& s) { cout << s; fout << s; };
    log("\n=== TABELA III: TEMPO DE CPU VS TAMANHO DO GRAFO ===\n");
    // MOD-05: Exibir número real de nós
    log("Nos_Reais\tArestas(k)\tEstat_media(ms)\tEstat_std\tTDSP_media(ms)\tTDSP_std\tOverhead(%)\n");

    int num_rodadas = 35;
    int warmup = 5;
    mt19937 rng(42);

    double soma_overhead = 0.0;
    int cont_pontos = 0;

    for (int k = 10; k <= 100; k += 10) {
        int side = (int)sqrt(k * 1000);
        int num_nodes = side * side;

        build_grid_graph(side, side, 1.5, rng);

        int source = 0;
        int target = num_nodes - 1;

        vector<double> tempos_estatico, tempos_dinamico;

        for (int r = 0; r < num_rodadas; r++) {
            double ms1, ms2;

            // MOD-03: Alternar ordem de execução
            if (r % 2 == 0) {
                auto start1 = chrono::high_resolution_clock::now();
                dijkstra_static(source, num_nodes);
                ms1 = chrono::duration<double, milli>(chrono::high_resolution_clock::now() - start1).count();

                auto start2 = chrono::high_resolution_clock::now();
                dijkstra_dynamic(source, num_nodes);
                ms2 = chrono::duration<double, milli>(chrono::high_resolution_clock::now() - start2).count();
            } else {
                auto start2 = chrono::high_resolution_clock::now();
                dijkstra_dynamic(source, num_nodes);
                ms2 = chrono::duration<double, milli>(chrono::high_resolution_clock::now() - start2).count();

                auto start1 = chrono::high_resolution_clock::now();
                dijkstra_static(source, num_nodes);
                ms1 = chrono::duration<double, milli>(chrono::high_resolution_clock::now() - start1).count();
            }

            if (r >= warmup) {
                tempos_estatico.push_back(ms1);
                tempos_dinamico.push_back(ms2);
            }
        }

        auto calc_stats = [](const vector<double>& v) -> pair<double, double> {
            double sum = accumulate(v.begin(), v.end(), 0.0);
            double mean = sum / v.size();
            double sq_sum = 0;
            for (auto x : v) sq_sum += (x - mean) * (x - mean);
            double std = sqrt(sq_sum / (v.size() - 1));
            return {mean, std};
        };

        auto [me, se] = calc_stats(tempos_estatico);
        auto [md, sd] = calc_stats(tempos_dinamico);

        // MOD-06: Média dos overheads
        double overhead = ((md - me) / me) * 100.0;
        soma_overhead += overhead;
        cont_pontos++;

        ostringstream oss;
        oss << num_nodes << "\t\t"
            << (total_edges_count / 1000) << "\t\t"
            << fixed << setprecision(2)
            << me << "\t\t" << se << "\t\t"
            << md << "\t\t" << sd << "\t\t"
            << overhead << "%\n";
        log(oss.str());
    }

    ostringstream oss_final;
    oss_final << "\nMedia Global de Overhead: " << fixed << setprecision(2) << (soma_overhead / cont_pontos) << "%\n";
    log(oss_final.str());
}

int main() {
    ios_base::sync_with_stdio(false);
    cin.tie(NULL);

    // MOD-04: Saída duplicada para rastreabilidade
    ofstream fout("resultados_v3.txt");

    simular_tabela_eficacia(fout);
    simular_tabela_cpu(fout);

    fout.close();
    return 0;
}