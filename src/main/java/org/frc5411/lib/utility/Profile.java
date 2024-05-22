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
//--------------------------------------------------------------------------[Package]--------------------------------------------------------------------------//
package org.frc5411.lib.utility;
//-----------------------------------------------------------------------[Libraries]----------------------------------------------------------------------------//
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.io.Closeable;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import lombok.NonNull;
//----------------------------------------------------------------------[Declaration]---------------------------------------------------------------------------//
/**
 * 
 * 
 * <h1>Difference</h1>
 * 
 * 
 * @author Cody Washington (@Jelatinone) 
 */
public class Profile<@NonNull Keybindings extends Enum<?>, @NonNull Preferences extends Enum<?>> implements Sendable, Closeable, Cloneable {
  //-----------------------------------------------------------------------[Constants]--------------------------------------------------------------------------//
  private static final List<Profile<?,?>> ALL_PROFILES = new ArrayList<>();
  private final Map<Keybindings, Trigger> KEYBINDING_MAP;
  private final Map<Preferences, Object> PREFERENCE_MAP;
  private final String NAME;
  //------------------------------------------------------------------------[Fields]----------------------------------------------------------------------------//
  private volatile SendableBuilder Builder;
  //---------------------------------------------------------------------[Constructor(s)]-----------------------------------------------------------------------//
  /**
   * Profile Constructor.
   */
  public Profile() {
    NAME = "";
    KEYBINDING_MAP = new HashMap<>();
    PREFERENCE_MAP = new HashMap<>();
  }

  /**
   * Profile Constructor.
   * @param Name Referenceable name to display this profile as
   */
  public Profile(final String Name) {
    NAME = Name;
    KEYBINDING_MAP = new HashMap<>();
    PREFERENCE_MAP = new HashMap<>();
  }
  //------------------------------------------------------------------------[Methods]---------------------------------------------------------------------------//
  /**
   * Adds a keybinding to the keybinding map of this profile, and updates that value across to the Sendable builder, if {@link #initSendable(SendableBuilder) applicable}. 
   * @param Keybinding Key to pull and put the Control value to
   * @param Control Trigger object to store within the Keybinding Map
   * @return This operator, for chained calls to {@link #add(Enum, Trigger)} and {@link #add(Enum, Object)}
   */
  public synchronized Profile<Keybindings,Preferences> add(final Keybindings Keybinding, final Trigger Control) {
    synchronized(KEYBINDING_MAP) {
      KEYBINDING_MAP.put(Keybinding, Control);
    }
    if(Builder != (null)) {
      Builder.addStringProperty(
        String.format(
          ("[%s]/Keybinding-[%s]"), 
          NAME, 
          Keybinding.name()), 
        () -> getKeybinding(Keybinding).get().toString(), 
        (final String Ignored) -> {});
      Builder.update();
    }
    return this;
  }

  /**
   * Adds a preference to the preference map of this profile, and updates that value across to the Sendable builder, if {@link #initSendable(SendableBuilder) applicable}. 
   * @param <Preference> Relevant type of the value being added to the preference map
   * @param Preference Key to pull and put the preference value to
   * @param Value Object of any type object to store within the Keybinding Map
   * @return This operator, for chained calls to {@link #add(Enum, Object)} and {@link #add(Enum, Trigger)}
   */
  public synchronized <Preference> Profile<Keybindings, Preferences> add(final Preferences Preference, final Preference Value) {
    synchronized(PREFERENCE_MAP) {
      PREFERENCE_MAP.put(Preference, Value);
    }
    if(Builder != (null)) {
      Builder.addStringProperty(
        String.format(
          ("[%s]/Keybinding-[%s]"), 
          NAME, 
          Preference.name()), 
        () -> getPreference(Preference).get().toString(), 
        (final String Ignored) -> {});
      Builder.update();
    }
    return this;
  }

  /**
   * Adds this Sendable instance to the sendable registry and publishes all currently present values within the maps.
   * @param Builder Object form which values can be pushed to and published
   */
  public synchronized void initSendable(final SendableBuilder Builder) {
    synchronized(this.Builder) {
      this.Builder = Builder;
      this.Builder.addStringProperty(
        NAME, 
        this::getName, 
        (final String Ignored) -> {});
      KEYBINDING_MAP.forEach((Keybinding, Control) -> 
        this.Builder.addStringProperty(
          Keybinding.name(), 
          () -> getKeybinding(Keybinding).get().toString(), 
          (final String Ignored) -> {}));
      PREFERENCE_MAP.forEach((Preference, Value) -> 
        this.Builder.addStringProperty(
          Preference.name(), 
          () -> getPreference(Preference).get().toString(), 
          (final String Ignored) -> {}));
      this.Builder.update();
    }
  }

  /**
   * Irreversibly clears all the map data for both Keybindings and Preferences, but doesn't specifically make this
   * instance unusable, and can be re-used with new keybindings.
   */
  @Override
  public synchronized void close() {
    synchronized(PREFERENCE_MAP) {
      PREFERENCE_MAP.clear();
    }
    synchronized(KEYBINDING_MAP) {
      KEYBINDING_MAP.clear();
    }
  }

  /**
   * Clones this profile instance, throws an exception when this method is called because a profile implicitly has only one instance
   * @return                            Nothing, an error is always thrown
   * @throws CloneNotSupportedException When the method is called, because a profile may only permit a single instance
   */
  @Override
  public final Object clone() throws CloneNotSupportedException {
    throw new CloneNotSupportedException(String.format(("[%s] Instances Cannot Be Cloned"), getClass().getCanonicalName()));
  }
  //-----------------------------------------------------------------------[Accessors]--------------------------------------------------------------------------//
  /**
   * Provides optionally a reference to the preference value, if present, of a preference within the preference map, null if not present
   * @param <Supplied> Type of the returned value, the value itself is cast to this type before it is made optional
   * @param Preference Key to pull the value from as an Object
   * @return {@link Optional} reference to a preference, if it exists or else it is {@link Optional#empty()}
   * @see {@link Optional}
   */
  @SuppressWarnings("unchecked")
  public <Supplied> Optional<Supplied> getPreference(final Preferences Preference) {
    return Optional.ofNullable((Supplied) PREFERENCE_MAP.get(Preference));
  }

  /**
   * Provides optionally a reference to the keybinding value, if present, of a preference within the keybinding map, null if not present
   * @param Keybinding Key to pull the value from as an Trigger
   * @return {@link Optional} reference to a keybinding, if it exists or else it is {@link Optional#empty()}
   * @see {@link Optional}
   */
  public Optional<Trigger> getKeybinding(final Keybindings Keybinding) {
    return Optional.ofNullable(KEYBINDING_MAP.get(Keybinding));
  }

  /**
   * Provides a list of all constructed {@link Profile} objects
   * @return List of profiles
   */
  public static List<Profile<?, ?>> getProfiles() {
    return ALL_PROFILES;
  }

  /**
   * Provides the name of this profile as a String
   * @return Name of this profile
   */
  public String getName() {
    return NAME;
  }
}
