function calcularDistancia(p1, p2) {
  const dx = p1[0] - p2[0];
  const dy = p1[1] - p2[1];
  return Math.sqrt(dx * dx + dy * dy);
}


function calcularLongitudRuta(ruta) {
  let total = 0;
  for (let i = 0; i < ruta.length - 1; i++) {
    total += calcularDistancia(ruta[i], ruta[i + 1]);
  }
  return total;
}

module.exports = { calcularDistancia, calcularLongitudRuta };
