package frc.robot.utility;

import com.fasterxml.jackson.annotation.JacksonAnnotationsInside;
import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.*;
import com.fasterxml.jackson.databind.annotation.JsonDeserialize;
import com.fasterxml.jackson.databind.annotation.JsonSerialize;
import com.fasterxml.jackson.databind.deser.ContextualDeserializer;
import com.fasterxml.jackson.databind.ser.ContextualSerializer;
import edu.wpi.first.math.util.Units;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

import java.io.IOException;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.util.Arrays;
import java.util.Set;

@JsonSerialize(using = UnitSerializer.class)
@JsonDeserialize(using = UnitDeserializer.class)
@JacksonAnnotationsInside
@Retention(RetentionPolicy.RUNTIME)
public @interface JsonUnits {
    Unit value();

    @RequiredArgsConstructor
    enum Unit {
        METERS("meters", Set.of("m")), FEET("feet", Set.of("ft")), INCHES("inches", Set.of("in")),
        RADIANS("radians", Set.of("rad")), DEGREES("degrees", Set.of("deg")), ROTATIONS("rotations", Set.of("rot")),
        METERS_PER_SECOND("meters per second", Set.of("m/s")), FEET_PER_SECOND("feet per second", Set.of("ft/s"));

        @Getter
        private final String preferredName;
        @Getter
        private final Set<String> alternateNames;
    }
}

class UnitSerializer extends JsonSerializer<Double> implements ContextualSerializer {
    private final JsonUnits.Unit unit;

    public UnitSerializer() {
        this(JsonUnits.Unit.METERS);
    }

    private UnitSerializer(JsonUnits.Unit unit) {
        this.unit = unit;
    }

    @Override
    public void serialize(Double value, JsonGenerator gen, SerializerProvider serializers) throws IOException {
        double convertedValue = Double.NaN;

        switch (unit) {
            case METERS:
            case RADIANS:
            case METERS_PER_SECOND:
                convertedValue = value;
                break;
            case FEET:
            case FEET_PER_SECOND:
                convertedValue = Units.metersToFeet(value);
                break;
            case INCHES:
                convertedValue = Units.metersToInches(value);
                break;
            case DEGREES:
                convertedValue = Units.radiansToDegrees(value);
                break;
            case ROTATIONS:
                convertedValue = Units.radiansToRotations(value);
                break;
        }

        gen.writeString(convertedValue + " " + unit.getPreferredName());
    }

    @Override
    public JsonSerializer<?> createContextual(SerializerProvider prov, BeanProperty property) {
        var units = property.getAnnotation(JsonUnits.class);

        return new UnitSerializer(units.value());
    }
}

class UnitDeserializer extends JsonDeserializer<Double> implements ContextualDeserializer {

    @Override
    public Double deserialize(JsonParser p, DeserializationContext ctxt) throws IOException, JsonProcessingException {
        String str = p.getValueAsString().trim();
        int sepIdx = str.indexOf(' ');
        if (sepIdx == -1) {
            throw new IOException("Unable to parse unit string '" + str + "'");
        }

        String valueStr = str.substring(0, sepIdx);
        String unitStr = str.substring(sepIdx + 1).trim();

        double value = Double.parseDouble(valueStr);
        JsonUnits.Unit unit = Arrays.stream(JsonUnits.Unit.values())
                .filter(u -> u.getPreferredName().equalsIgnoreCase(unitStr) || u.getAlternateNames().contains(unitStr.toLowerCase()))
                .findFirst().orElseThrow(() -> new IOException("Unable to parse unit suffix '" + unitStr + "'"));

        switch (unit) {
            case METERS:
            case METERS_PER_SECOND:
            case RADIANS:
                return value;
            case FEET:
            case FEET_PER_SECOND:
                return Units.feetToMeters(value);
            case INCHES:
                return Units.inchesToMeters(value);
            case DEGREES:
                return Units.degreesToRadians(value);
            case ROTATIONS:
                return Units.rotationsToRadians(value);
        }
        return 0.0;
    }

    @Override
    public JsonDeserializer<?> createContextual(DeserializationContext ctxt, BeanProperty property) {
        return this;
    }
}