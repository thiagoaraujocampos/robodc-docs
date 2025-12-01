import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Sidebar do projeto RobôDC
 * Estrutura organizada por seções principais
 */
const sidebars: SidebarsConfig = {
  docsSidebar: [
    'intro',
    'visao-geral',
    {
      type: 'category',
      label: 'Robô de 1ª Geração',
      items: [
        'robo-1gen/visao-geral',
        'robo-1gen/hardware',
        'robo-1gen/instalacao',
        'robo-1gen/estrutura-repositorio',
        {
          type: 'category',
          label: 'Software',
          items: [
            'robo-1gen/software/arquitetura-ros1',
            'robo-1gen/software/funcionalidades',
          ],
        },
        'robo-1gen/implantacao',
      ],
    },
    {
      type: 'category',
      label: 'Robô de 2ª Geração',
      items: [
        'robo-2gen/visao-geral',
        {
          type: 'category',
          label: 'Começando (2ª Geração)',
          items: [
            'robo-2gen/comecando/prerequisitos',
            'robo-2gen/comecando/instalacao-execucao',
            'robo-2gen/comecando/estrutura-repositorio',
          ],
        },
        'robo-2gen/hardware',
        {
          type: 'category',
          label: 'Software',
          items: [
            'robo-2gen/software/arquitetura-ros2',
          ],
        },
        'robo-2gen/comunicacao',
      ],
    },
    {
      type: 'category',
      label: 'Operação do Sistema',
      items: [
        'operacao/operacao-1gen',
        'operacao/operacao-2gen',
        'operacao/procedimentos-seguranca',
        'operacao/limitacoes',
      ],
    },
    {
      type: 'category',
      label: 'Metodologia Experimental',
      items: [
        'metodologia/metodologia-experimentacao',
      ],
    },
    {
      type: 'category',
      label: 'Desenvolvimento no Projeto',
      items: [
        'desenvolvimento/como-contribuir',
      ],
    },
    {
      type: 'category',
      label: 'Referências e Materiais',
      items: [
        'referencias/documentos',
        'referencias/bibliografia',
        'referencias/links-uteis',
      ],
    },
  ],
};

export default sidebars;
