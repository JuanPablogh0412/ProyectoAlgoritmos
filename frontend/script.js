// Inicializar mapa
const map = L.map('map').setView([4.6021, -74.0652], 13);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);

// Variables globales
let nodes = [];
let edges = [];
let drawnEdges = [];
const POI_PREFIX = 'POI_';
let graph = {};
let distanceMatrix = null;
let currentRoutes = {};

// ================== CARGAR RED VIAL ==================
document.getElementById('roadNetworkFile').addEventListener('change', function(e) {
    const file = e.target.files[0];
    const reader = new FileReader();

    reader.onload = function(e) {
        resetData();
        processNetworkData(e.target.result);
        buildGraph();
        drawNetwork();
    };
    reader.readAsText(file);
});

function resetData() {
    nodes = [];
    edges = [];
    graph = {};
    currentRoutes = {};
    distanceMatrix = null;
    drawnEdges.forEach(edge => map.removeLayer(edge));
    drawnEdges = [];
    document.getElementById('resultsBody').innerHTML = '';
}

function processNetworkData(data) {
    const lines = data.split('\n').map(line => line.trim());
    let isNodeSection = false;
    let isEdgeSection = false;

    lines.forEach(line => {
        if (line.startsWith('node_id')) {
            isNodeSection = true;
            isEdgeSection = false;
            return;
        }
        if (line.startsWith('edge_id')) {
            isEdgeSection = true;
            isNodeSection = false;
            return;
        }

        const parts = line.split(',');
        if (isNodeSection && parts.length === 3) {
            nodes.push({ id: parts[0].trim(), lat: parseFloat(parts[1]), lon: parseFloat(parts[2]) });
        } else if (isEdgeSection && parts.length === 3) {
            edges.push({ id: parts[0].trim(), start: parts[1].trim(), end: parts[2].trim() });
        }
    });
}

// ================== INTEGRAR PUNTOS SOBRE ARISTA ==================
document.getElementById('pointsetFile').addEventListener('change', function(e) {
    const file = e.target.files[0];
    const reader = new FileReader();

    reader.onload = function(e) {
        processPointsData(e.target.result);
        updateDistanceMatrix();
    };
    reader.readAsText(file);
});

function processPointsData(data) {
    const points = data.split('\n')
        .map(line => line.trim())
        .filter(line => !line.startsWith('point_id') && line)
        .map(line => {
            const [id, lat, lon] = line.split(',');
            return { id: id.trim(), lat: parseFloat(lat), lon: parseFloat(lon) };
        });

    points.forEach(point => integratePoint(point));
}

function integratePoint(point) {
    const pt = turf.point([point.lon, point.lat]);
    let closestEdge = null;
    let minDistance = Infinity;
    let snappedPoint = null;

    // Encontrar arista más cercana y punto proyectado
    edges.forEach(edge => {
        const startNode = nodes.find(n => n.id === edge.start);
        const endNode = nodes.find(n => n.id === edge.end);
        if (!startNode || !endNode) return;

        const line = turf.lineString([
            [startNode.lon, startNode.lat],
            [endNode.lon, endNode.lat]
        ]);
        const snapped = turf.nearestPointOnLine(line, pt);
        if (snapped.properties.dist < minDistance) {
            minDistance = snapped.properties.dist;
            closestEdge = edge;
            snappedPoint = snapped.geometry.coordinates; // [lon, lat]
        }
    });

    if (closestEdge && snappedPoint) {
        // Crear nodo en la posición proyectada
        const newNode = createNewNodeOnEdge(snappedPoint);

        // Partir arista original en dos
        updateEdges(closestEdge, newNode);

        // Redibujar aristas afectadas
        redrawAffectedEdges(closestEdge);

        // Dibujar nodo proyectado
        drawNewNodePunto(newNode);

        // Reconstruir grafo con nuevo nodo
        buildGraph();
    }
}

function createNewNodeOnEdge(coords) {
    const [lon, lat] = coords;
    const newNode = { id: `${POI_PREFIX}${nodes.length + 1}`, lat, lon };
    nodes.push(newNode);
    return newNode;
}

function updateEdges(originalEdge, newNode) {
    // Eliminar original
    edges = edges.filter(e => e.id !== originalEdge.id);
    // Añadir dos nuevas particiones
    edges.push(
        { id: `${originalEdge.id}_A`, start: originalEdge.start, end: newNode.id },
        { id: `${originalEdge.id}_B`, start: newNode.id, end: originalEdge.end }
    );
}

function redrawAffectedEdges(originalEdge) {
    const affectedIds = [originalEdge.id, `${originalEdge.id}_A`, `${originalEdge.id}_B`];
    drawnEdges = drawnEdges.filter(poly => {
        if (affectedIds.includes(poly.options.edgeId)) {
            map.removeLayer(poly);
            return false;
        }
        return true;
    });
    edges.filter(e => affectedIds.includes(e.id)).forEach(edge => {
        const s = nodes.find(n => n.id === edge.start);
        const t = nodes.find(n => n.id === edge.end);
        if (!s || !t) return;
        const poly = L.polyline([[s.lat, s.lon], [t.lat, t.lon]], { color: 'gray', edgeId: edge.id }).addTo(map);
        drawnEdges.push(poly);
    });
}

function drawNewNodePunto(node) {
    L.marker([node.lat, node.lon], {
        icon: L.icon({
            iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-green.png',
            iconSize: [25, 41]
        })
    }).addTo(map).bindPopup(`Punto ${node.id}`);
}

// ================== ALGORITMOS ================== //
document.getElementById('runBruteForce').addEventListener('click', async () => {
    try {
        const pois = getPOIs();
        const result = await executeAlgorithm('Fuerza Bruta', () => bruteForceTSP(pois, distanceMatrix));
        handleAlgorithmResult(result, '#FF0000');
    } catch (error) {
        handleError('Fuerza Bruta', error);
    }
});

document.getElementById('runNearest').addEventListener('click', async () => {
    const pois = getPOIs();
    const matrix = distanceMatrix;
    const result = await executeAlgorithm('Nearest Neighbor', () => nearestNeighborTSP(pois, matrix));
    handleAlgorithmResult(result, '#0000FF');
});

document.getElementById('runGenetic').addEventListener('click', async () => {
    try {
        const pois = getPOIs();
        const genetic = new GeneticAlgorithm();
        const result = await executeAlgorithm('Genético', () => genetic.evolve(pois, distanceMatrix));
        handleAlgorithmResult(result, '#00FF00');
    } catch (error) {
        handleError('Genético', error);
    }
});

// ================== FUNCIONES DEL SISTEMA ================== //
function buildGraph() {
    graph = {};
    edges.forEach(edge => {
        const startNode = nodes.find(n => n.id === edge.start);
        const endNode = nodes.find(n => n.id === edge.end);
        if (!startNode || !endNode) return;

        const distance = calculateDistance(startNode, endNode);
        
        if (!graph[edge.start]) graph[edge.start] = {};
        if (!graph[edge.end]) graph[edge.end] = {};
        
        graph[edge.start][edge.end] = distance;
        graph[edge.end][edge.start] = distance;
    });
}

function updateDistanceMatrix() {
    const pois = nodes.filter(node => node.id.startsWith(POI_PREFIX));
    if (pois.length >= 2) {
        distanceMatrix = createDistanceMatrix(pois);
    }
}

function createDistanceMatrix(pois) {
    const matrix = {};
    pois.forEach(poi1 => {
        matrix[poi1.id] = {};
        const { distances } = dijkstra(poi1.id);
        pois.forEach(poi2 => matrix[poi1.id][poi2.id] = distances[poi2.id] || Infinity);
    });
    return matrix;
}

// ================== ALGORITMO DIJKSTRA ================== //
function dijkstra(startId) {
    const distances = {};
    const previous = {};
    const unvisited = new Set(nodes.map(n => n.id));

    nodes.forEach(node => {
        distances[node.id] = Infinity;
        previous[node.id] = null;
    });
    distances[startId] = 0;

    while (unvisited.size > 0) {
        let currentId = Array.from(unvisited).reduce((minId, nodeId) => 
            distances[nodeId] < distances[minId] ? nodeId : minId
        , unvisited.values().next().value);

        if (distances[currentId] === Infinity) break;
        
        unvisited.delete(currentId);
        Object.entries(graph[currentId] || {}).forEach(([neighborId, dist]) => {
            const alt = distances[currentId] + dist;
            if (alt < distances[neighborId]) {
                distances[neighborId] = alt;
                previous[neighborId] = currentId;
            }
        });
    }

    return { distances, previous };
}

function getShortestPath(startId, endId) {
    const { previous } = dijkstra(startId);
    const path = [];
    let currentId = endId;

    while (currentId && currentId !== startId) {
        path.unshift(currentId);
        currentId = previous[currentId];
    }

    if (currentId === startId) {
        path.unshift(startId);
        return path;
    }
    return null;
}


// ================== ALGORITMO GENÉTICO ==================
class GeneticAlgorithm {
  constructor(populationSize = 50, mutationRate = 0.01, maxGenerations = 100) {
    this.populationSize = populationSize;
    this.mutationRate = mutationRate;
    this.maxGenerations = maxGenerations;
  }

  // Función principal: evolución hasta maxGenerations
  evolve(pois, distanceMatrix) {
    let population = this.initializePopulation(pois, distanceMatrix);
    for (let gen = 0; gen < this.maxGenerations; gen++) {
      population = this.nextGeneration(population, distanceMatrix);
    }
    const best = this.getBestSolution(population);
    return {
      path: [...best.route, best.route[0]], // cerramos el ciclo
      distance: best.distance
    };
  }

  // Crea la población inicial con rutas aleatorias
  initializePopulation(pois, distanceMatrix) {
    const pop = [];
    for (let i = 0; i < this.populationSize; i++) {
      const route = [...pois].sort(() => Math.random() - 0.5);
      pop.push({
        route,
        distance: this.calculateRouteDistance(route, distanceMatrix)
      });
    }
    return pop;
  }

  // Calcula la distancia total de una ruta circular
  calculateRouteDistance(route, distanceMatrix) {
    let dist = 0;
    for (let i = 0; i < route.length - 1; i++) {
      dist += distanceMatrix[route[i].id][route[i + 1].id];
    }
    // regreso al inicio
    dist += distanceMatrix[route[route.length - 1].id][route[0].id];
    return dist;
  }

  // Genera la siguiente generación: selección, cruce y mutación
  nextGeneration(population, distanceMatrix) {
    const newPop = [];
    while (newPop.length < this.populationSize) {
      const parentA = this.tournamentSelection(population);
      const parentB = this.tournamentSelection(population);
      let child = this.crossover(parentA, parentB);
      child = this.mutate(child, distanceMatrix);
      newPop.push(child);
    }
    return newPop;
  }

  // Selección por torneo entre dos individuos aleatorios
  tournamentSelection(population) {
    const a = population[Math.floor(Math.random() * population.length)];
    const b = population[Math.floor(Math.random() * population.length)];
    return (a.distance < b.distance) ? a : b;
  }

  // Cruce OX (Order Crossover)
  crossover(parentA, parentB) {
    const len = parentA.route.length;
    const start = Math.floor(Math.random() * len);
    const end = start + Math.floor(Math.random() * (len - start));
    const slice = parentA.route.slice(start, end);
    const childRoute = [
      ...slice,
      ...parentB.route.filter(node => !slice.includes(node))
    ];
    return { route: childRoute, distance: null };
  }

  // Mutación por swap aleatorio con cierta probabilidad
  mutate(individual, distanceMatrix) {
    if (Math.random() < this.mutationRate) {
      const i = Math.floor(Math.random() * individual.route.length);
      const j = Math.floor(Math.random() * individual.route.length);
      [individual.route[i], individual.route[j]] = [individual.route[j], individual.route[i]];
    }
    individual.distance = this.calculateRouteDistance(individual.route, distanceMatrix);
    return individual;
  }

  // Devuelve el mejor individuo de la población
  getBestSolution(population) {
    return population.reduce((best, curr) =>
      (curr.distance < best.distance) ? curr : best
    , population[0]);
  }
}


// ================== FUNCIONES TSP ================== //
function bruteForceTSP(pois, matrix) {
    if (pois.length > 10) throw new Error("Máximo 10 puntos para fuerza bruta");
    
    const permutations = generatePermutations(pois);
    let best = { distance: Infinity };

    permutations.forEach(perm => {
        let distance = calculatePathDistance(perm, matrix);
        if (distance < best.distance) {
            best = { path: [...perm, perm[0]], distance };
        }
    });

    return best;
}

function generatePermutations(arr) {
    const result = [];
    const permute = (arr, m = []) => {
        if (arr.length === 0) result.push(m);
        else arr.forEach((el, i) => 
            permute([...arr.slice(0, i), ...arr.slice(i + 1)], [...m, el])
        );
    };
    permute(arr);
    return result;
}

function nearestNeighborTSP(pois, matrix) {
    const unvisited = new Set(pois.map(p => p.id));
    const start = pois[0].id;
    let current = start;
    const route = [pois.find(p => p.id === start)];
    unvisited.delete(start);

    while (unvisited.size > 0) {
        let nearest = null;
        let minDist = Infinity;
        unvisited.forEach(id => {
            if (matrix[current][id] < minDist) {
                minDist = matrix[current][id];
                nearest = id;
            }
        });
        route.push(pois.find(p => p.id === nearest));
        unvisited.delete(nearest);
        current = nearest;
    }
    // cerrar ciclo
    route.push(pois.find(p => p.id === start));
    const distance = calculatePathDistance(route.slice(0, -1), matrix) + matrix[current][start];
    return { path: route, distance };
}

// ================== MANEJO DE RESULTADOS ================== //
async function executeAlgorithm(name, algorithm) {
    const start = performance.now();
    const result = await algorithm();
    return {
        name,
        path: result.path,
        distance: result.distance,
        time: performance.now() - start,
        layer: null
    };
}

function handleAlgorithmResult(result, color) {
    // Eliminar ruta anterior del mismo tipo
    if (currentRoutes[result.name]) {
        map.removeLayer(currentRoutes[result.name].layer);
    }

    // Dibujar nueva ruta
    const pathCoordinates = [];
    for (let i = 0; i < result.path.length - 1; i++) {
        const segmentPath = getShortestPath(result.path[i].id, result.path[i + 1].id);
        if (segmentPath) {
            segmentPath.forEach(nodeId => {
                const node = nodes.find(n => n.id === nodeId);
                pathCoordinates.push([node.lat, node.lon]);
            });
        }
    }

    const routeLayer = L.polyline(pathCoordinates, {
        color: color,
        weight: 3,
        className: `route-${result.name.toLowerCase()}`
    }).addTo(map);

    // Actualizar registro
    currentRoutes[result.name] = {
        ...result,
        layer: routeLayer
    };

    // Actualizar tabla
    updateResultsTable(result);
}

function updateResultsTable(result) {
    const existingRow = document.querySelector(`tr[data-algorithm="${result.name}"]`);
    const rowHtml = `
        <td>${result.name}</td>
        <td>${result.distance.toFixed(2)}</td>
        <td>${result.time.toFixed(2)}</td>
        <td>
            <button onclick="toggleRoute('${result.name}')">Mostrar/Ocultar</button>
            <button onclick="removeResult('${result.name}')">Eliminar</button>
        </td>
    `;

    if (existingRow) {
        existingRow.innerHTML = rowHtml;
    } else {
        const row = document.createElement('tr');
        row.setAttribute('data-algorithm', result.name);
        row.innerHTML = rowHtml;
        document.getElementById('resultsBody').appendChild(row);
    }
}

// ================== FUNCIONES DE INTERFAZ ================== //
window.toggleRoute = function(algorithmName) {
    const route = currentRoutes[algorithmName];
    if (route) {
        const visibility = route.layer.options.opacity === 0 ? 1 : 0;
        route.layer.setStyle({ opacity: visibility, fillOpacity: visibility });
    }
};

window.removeResult = function(algorithmName) {
    if (currentRoutes[algorithmName]) {
        map.removeLayer(currentRoutes[algorithmName].layer);
        delete currentRoutes[algorithmName];
    }
    document.querySelector(`tr[data-algorithm="${algorithmName}"]`)?.remove();
};

function handleError(algorithmName, error) {
    console.error(`Error en ${algorithmName}:`, error);
    alert(`${algorithmName}: ${error.message}`);
}

function getPOIs() {
    const pois = nodes.filter(node => node.id.startsWith(POI_PREFIX));
    if (pois.length < 2) throw new Error("Se requieren al menos 2 puntos de interés");
    return pois;
}

function calculateDistance(node1, node2) {
    return turf.distance(
        turf.point([node1.lon, node1.lat]),
        turf.point([node2.lon, node2.lat])
    );
}

function calculatePathDistance(path, matrix) {
    return path.reduce((total, node, i) => 
        total + (i > 0 ? matrix[path[i-1].id][node.id] : 0), 0
    );
}
// ================== DIBUJAR RED ================== //
function drawNetwork() {
    // Dibujar nodos
    nodes.forEach(node => {
        L.marker([node.lat, node.lon], {
            icon: L.icon({
                iconUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/images/marker-icon.png',
                iconSize: [25, 41]
            })
        }).addTo(map);
    });

    // Dibujar aristas
    edges.forEach(edge => {
        const startNode = nodes.find(n => n.id === edge.start);
        const endNode = nodes.find(n => n.id === edge.end);
        if (startNode && endNode) {
            L.polyline(
                [[startNode.lat, startNode.lon], [endNode.lat, endNode.lon]],
                { color: '#666', weight: 2 }
            ).addTo(map);
        }
    });
}
// ================== DESCARGA DE RESULTADOS ================== //
document.getElementById('downloadResults').addEventListener('click', () => {
    const geojson = {
        type: 'FeatureCollection',
        features: [
            ...nodes.map(node => ({
                type: 'Feature',
                geometry: { type: 'Point', coordinates: [node.lon, node.lat] },
                properties: { id: node.id, type: 'node' }
            })),
            ...edges.map(edge => ({
                type: 'Feature',
                geometry: {
                    type: 'LineString',
                    coordinates: [
                        [nodes.find(n => n.id === edge.start).lon, nodes.find(n => n.id === edge.start).lat],
                        [nodes.find(n => n.id === edge.end).lon, nodes.find(n => n.id === edge.end).lat]
                    ]
                },
                properties: { id: edge.id, type: 'edge' }
            }))
        ]
    };

    const blob = new Blob([JSON.stringify(geojson)], { type: 'application/json' });
    const link = document.createElement('a');
    link.href = URL.createObjectURL(blob);
    link.download = 'red-actualizada.geojson';
    link.click();
});