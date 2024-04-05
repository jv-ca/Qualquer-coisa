import fluids
class Agua:
    def __init__(self, temperatura):
        self.temperatura = temperatura
    
    def viscosidade(self):
        # Implementação da viscosidade da água em função da temperatura
        # Aqui estamos apenas retornando um valor fixo para fins de exemplo
        # Você pode adicionar sua própria lógica para calcular a viscosidade com base na temperatura
        return 0.001  # Viscosidade da água a 20°C
    
    def densidade(self):
        # A densidade da água é fixa como 1000 kg/m^3
        return 1000  # Densidade da água em qualquer temperatura é 1000 kg/m^3

# Criando uma instância da classe Agua com uma temperatura específica
temperatura_agua = 0  # Temperatura em graus Celsius
agua = Agua(temperatura_agua)

# Acessando a viscosidade da água
viscosidade_agua = agua.viscosidade()

# Acessando a densidade da água
densidade_agua = agua.densidade()

print(f"Para uma temperatura de {temperatura_agua}°C:")
print(f"Viscosidade da água: {viscosidade_agua:.6f} Pa*s")
print(f"Densidade da água: {densidade_agua} kg/m^3")
