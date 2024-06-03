//------------------------------------------------------------------------[License]----------------------------------------------------------------------------//
// Copyright 2024 Cody Washington
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//------------------------------------------------------------------------[Package]----------------------------------------------------------------------------//
package org.frc5411.lib.annotation;
//-----------------------------------------------------------------------[Libraries]---------------------------------------------------------------------------//
import org.checkerframework.checker.nullness.qual.NonNull;
import org.checkerframework.checker.units.qual.UnknownUnits;
import org.checkerframework.framework.qual.SubtypeOf;
import org.checkerframework.framework.qual.TypeKind;
import org.checkerframework.framework.qual.UpperBoundFor;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
//----------------------------------------------------------------------[Declaration]--------------------------------------------------------------------------//
/**
 * <h1>Unit</h1>
 * 
 * <p>A marker annotation which is used to symbolized that a numeric value is a measured or measurable type, and is assigned with a specific 
 * type that describes the type of measurement value, i.e. A directional value that represents the angle of a device in radians. 
 * 
 * <p>Marker annotation which is used purely to symbol that a given numeric value (i.e integer, float, double, long, etc) is a measurable value such as
 * an angle, velocity, etc. and should be handled as such in all relevant calculations.
 * 
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.FIELD)
@UpperBoundFor(typeKinds = {
    TypeKind.TYPEVAR,
    TypeKind.INT,
    TypeKind.DOUBLE,
    TypeKind.FLOAT,
    TypeKind.LONG,
    TypeKind.SHORT,
    TypeKind.BYTE
  })
@Documented
@SubtypeOf(UnknownUnits.class)
public @interface Unit {
  //------------------------------------------------------------------------[Methods]--------------------------------------------------------------------------//
  /**
   * Determines the type of measurement that has been performed to source this number, i.e. what has been measured, but not necessarily
   * it's relevant units (such as radians, degrees, and rotations all being relevant measurements of angle).
   * @return Measured type enum value
   */
  @NonNull Measured measures();

  /**
   * Determines the units of the measured value, optional (and unenforced) parameter; has no connection to the measured value.
   * @return String name (any format is acceptable) of the symbol of the measured value
   */
  @NonNull String symbol() default ("");
  //-----------------------------------------------------------------------[Internal]--------------------------------------------------------------------------//
  /**
   * <h1> Measured </h1>
   * 
   * <p>Enum consisting of measurable values (but notably not their units)
   */
  enum Measured {
    AMBIGUOUS,
    ANGLE,
    VELOCITY,
    ACCELERATION,
    DISTANCE,
    VOLUME,
    AREA,
    TIME,
    VOLTAGE,
    AMPERAGE,
  }  
}