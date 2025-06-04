const { calcularDistancia } = require('./tspUtils');

describe('calcularDistancia', () => {
  it('debería retornar 0 si los puntos son iguales', () => {
    expect(calcularDistancia([0, 0], [0, 0])).toBe(0);
  });

  it('debería calcular distancia euclidiana', () => {
    expect(calcularDistancia([0, 0], [3, 4])).toBe(5); // 3-4-5
  });
});
