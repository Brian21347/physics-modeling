import argparse

from solver import Solver


def main():
    parser = argparse.ArgumentParser(
        description="Solver for the `Routing on Curved Worlds` prompt for the SPS Modeling competition."
    )
    parser.add_argument("-s", "--seed", type=int, help="Seed set for generating the worlds")
    parser.add_argument("-o", "--output", type=str, help="Path to the output .json file")
    parser.add_argument(
        "-v", "--verbose", action="store_true", help="Increase output verbosity for debugging"
    )
    args = parser.parse_args()
    sol = Solver(args.seed)
    sol.solve()
    sol.save(args.output)


if __name__ == "__main__":
    main()
