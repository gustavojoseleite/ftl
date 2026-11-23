from setuptools import setup

package_name = 'ftl_analysis'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gustavo',
    maintainer_email='gujoseleite@gmail.com',
    description=(
        'Análise das campanhas do ftl: lê os bags de cada execução e gera as métricas por '
        'trecho e por trator, com média e intervalo de confiança de 95%, em tabelas e figura.'),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'analyze = ftl_analysis.report:main',
            'bag_info = ftl_analysis.bag:main',
            'plot_paths = ftl_analysis.plot:main',
        ],
    },
)
