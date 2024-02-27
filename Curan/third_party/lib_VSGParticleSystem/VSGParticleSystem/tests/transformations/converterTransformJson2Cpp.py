import json

if __name__ == "__main__":
    with open("2023_10_04_demo_v2_gg-3D-test.json", 'r') as tf:
        data = json.load(tf)

    sourceFile = """#include "TransformPoint.h"

std::vector<TransformPoint> getTransformData() {
    return {"""
    sourceFile += "\n\t\t"

    xiWorld = data["xiWorld"]
    xiDisp = data["xiDisp"]
    xiForce = data["xiForce"]
    xiDeformed = data["xiDeformed"]
    xiNormalized = data["xiNormalized"]
    dxiWorld = data["dxiWorld"]
    dxiDisp = data["dxiDisp"]
    dxiForce = data["dxiForce"]
    dxiDeformed = data["dxiDeformed"]
    dxiNormalized = data["dxiNormalized"]
    line = ""
    i = 0
    for xw, xdi, xf, xde, xn, dxn, dxde, dxf, dxdi, dxw in zip(xiWorld, xiDisp, xiForce, xiDeformed, xiNormalized, dxiNormalized, dxiDeformed, dxiForce, dxiDisp, dxiWorld):
        line = r"{{" + f"{xw}".replace('[', '').replace(']', '') + r"}, "
        line += r"{" + f"{xdi}".replace('[', '').replace(']', '') + r"}, "
        line += r"{" + f"{xf}".replace('[', '').replace(']', '') + r"}, "
        line += r"{" + f"{xde}".replace('[', '').replace(']', '') + r"}, "
        line += r"{" + f"{xn}".replace('[', '').replace(']', '') + r"}, "
        line += r"{" + f"{dxn}".replace('[', '').replace(']', '') + r"}, "
        line += r"{" + f"{dxde}".replace('[', '').replace(']', '') + r"}, "
        line += r"{" + f"{dxf}".replace('[', '').replace(']', '') + r"}, "
        line += r"{" + f"{dxdi}".replace('[', '').replace(']', '') + r"}, "
        line += r"{" + f"{dxw}".replace('[', '').replace(']', '') + r"}},"
        line += "\n\t\t"
        sourceFile += line
        i += 1
        if i > 32500: break

    sourceFile = sourceFile[:-3] + "};\n}"

    with open("TransformPoint.cpp", 'w') as tf:
        tf.write(sourceFile)