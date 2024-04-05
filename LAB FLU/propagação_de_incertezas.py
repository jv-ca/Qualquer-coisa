import math

#---------------------LEITURA DO TXT-------------------------#
#abrir o arquivo txt no modo leitura
arquivo = open('teste_P_I.txt', 'r') 
#ler as linhas e armazenar cada linha em um elemento de uma lista, considerando todos os caracteres
linhas = arquivo.readlines() 
#editar as linhas lidas em outra lista, dessa vez com formatação
linha_processada = [linha.strip() for linha in linhas]
#verificação do que foi lido


#--------------------ATRIBUIÇÃO DE DADOS-----------------------#
#número de médidas, método split divide a frase original em 2, e com a index, atribui-se o 2° elemento
numero_de_medidas = int(linha_processada[0].split(": ")[1])
print(numero_de_medidas)

"""for linha in linha_processada[1:int(numero_de_medidas)+1]:
    medidas = linha.split(': ')[1]
    print(medidas)
"""
def media_medidas():
    medidas = linha_processada[1:(numero_de_medidas+1)]
    resultado = sum(medidas)/int(numero_de_medidas)
    return resultado

media_medidas()