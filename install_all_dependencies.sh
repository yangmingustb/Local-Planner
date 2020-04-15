set -e
CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "install eigen3"
bash ${CURRENT_DIR}/install_eigen3.sh

echo "install map_server"
bash ${CURRENT_DIR}/install_map_server.sh

echo "install ompl"
bash ${CURRENT_DIR}/install_ompl.sh

echo "install armadillo"
bash ${CURRENT_DIR}/install_armadillo.sh

echo "install nlopt"
bash ${CURRENT_DIR}/install_nlopt.sh