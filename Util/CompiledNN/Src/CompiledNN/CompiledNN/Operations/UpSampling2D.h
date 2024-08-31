/**
 * @author Arne Hasselbring
 */

#pragma once

#include "../CompiledNNImplBase.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    struct UpSampling2DCompiler : public SISOOperationCompiler
    {
      struct Parameters final
      {
        std::array<unsigned int, 2> size;
        InterpolationMethod method;

        bool operator==(const Parameters& other) const
        {
          return size == other.size &&
                 method == other.method;
        }
      };
      const Parameters p;

      UpSampling2DCompiler(const CompilationSettings& settings, const Parameters& p) : SISOOperationCompiler(settings), p(p) {}

      inline bool canBeInplace() const override
      {
        return p.size[0] == 1 && p.size[1] == 1;
      }

      void initialize() override {}
      void compile(x86::Assembler& a, ActivationFunctionHandler& afHandler, const TensorPointerXf& input, const TensorPointerXf& output) const override;

      inline std::vector<unsigned int> calcOutputDimensions(const std::vector<unsigned int>& inputDimensions) const override
      {
        ASSERT(inputDimensions.size() == 3);
        return {{inputDimensions[0] * p.size[0], inputDimensions[1] * p.size[1], inputDimensions[2]}};
      }
    };
  }
}
