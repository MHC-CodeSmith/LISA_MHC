# Estrutura de ROS para Lisa

## Visão Geral
Esta branch contém a implementação da estrutura de comunicação do ROS utilizada no projeto Lisa, focando na interação entre os nós através de publishers e subscribers. O código aqui presente estabelece como diferentes componentes do sistema comunicam-se, permitindo um fluxo de dados eficiente e organizado.

## Estrutura de Comunicação
A estrutura de comunicação é baseada em tópicos do ROS, onde diferentes nós publicam e se inscrevem em tópicos específicos para trocar informações. Isso inclui:

- **Publicadores (Publishers):** Nós que enviam dados, como imagens de vídeo ou informações sensoriais.
- **Subscritores (Subscribers):** Nós que recebem dados de outros nós para processamento ou ação subsequente.

Esses componentes são essenciais para o funcionamento autônomo e a interatividade da Lisa, permitindo que o sistema responda de maneira dinâmica ao ambiente e aos comandos do usuário.

## Fluxograma da Estrutura
Incluímos um fluxograma detalhado para ilustrar como os componentes interagem dentro da rede ROS. Para visualizar este fluxograma, navegue até a pasta de imagens neste repositório e abra o arquivo `fluxograma.png`.

## Como Usar
Para utilizar o código nesta branch, siga os passos abaixo:
1. **Clone o Repositório:**
   ```bash
   git clone [URL-do-repositório]
   git checkout [nome-da-branch]
