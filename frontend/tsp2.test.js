const { calcularDistancia, calcularLongitudRuta } = require('./tspUtils');

describe('calcularLongitudRuta', () => {
  it('debería retornar 0 si la ruta tiene un solo punto', () => {
    const ruta = [[0, 0]];
    expect(calcularLongitudRuta(ruta)).toBe(0);
  });

  it('debería retornar la distancia correcta entre dos puntos', () => {
    const ruta = [[0, 0], [3, 4]];
    expect(calcularLongitudRuta(ruta)).toBe(5); // 3-4-5 triangle
  });

  it('debería calcular la distancia total para múltiples puntos', () => {
    const ruta = [[0, 0], [3, 4], [6, 8]]; // 5 + 5 = 10
    expect(calcularLongitudRuta(ruta)).toBe(10);
  });
});
